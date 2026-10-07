"""Outward bounds for flange motion over a complete independent joint box.

This is geometry only: no device, cache, interpolation, attachment or permission
is inferred. Inputs describe the exact supplied binary floats. All operations
used by the bound, including central trigonometry, are enclosed with directed
Decimal arithmetic; outputs round outward to binary floats. Supported MDH
lengths are at most 10 m and individual angles at most 32 rad (q+offset: 64).
The caller retains its separate hold budget and whole-body clearance bound.
"""
from decimal import Context, Decimal, ROUND_CEILING, ROUND_FLOOR, ROUND_HALF_EVEN
from functools import lru_cache
import math
import struct


PRECISION = 70
TAYLOR_TERMS = 128
_DN = Context(prec=PRECISION, rounding=ROUND_FLOOR)
_UP = Context(prec=PRECISION, rounding=ROUND_CEILING)
_RN = Context(prec=PRECISION, rounding=ROUND_HALF_EVEN)
_Z, _ONE, _TWO = Decimal(0), Decimal(1), Decimal(2)
_ZERO, _UNIT = (_Z, _Z), (_ONE, _ONE)


def _decimal(value, name, *, maximum=None, nonnegative=False):
    if type(value) not in (int, float):
        raise ValueError(name + " must be a finite number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite or (nonnegative and value < 0) or (maximum is not None and abs(value) > maximum):
        raise ValueError(name + " outside supported finite range")
    return Decimal.from_float(value) if type(value) is float else Decimal(value)


def _vector(values, name, maximum):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError(name + " must contain six numbers")
    return tuple(_decimal(v, name, maximum=maximum) for v in values)


def _next_up(value):
    """Positive finite binary64 successor, also available on Python 3.8."""
    bits = struct.unpack(">Q", struct.pack(">d", value))[0]
    return struct.unpack(">d", struct.pack(">Q", bits+1))[0]


def _float_upper(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Bound cannot be represented as a finite float")
    if Decimal.from_float(result) < value:
        result = _next_up(result)
    if not math.isfinite(result):
        raise ValueError("Bound cannot be represented as a finite float")
    return result


def sum_upper(values):
    """Finite nonnegative sum rounded upward, without losing the last ulp."""
    total = _Z
    for value in values:
        total = _UP.add(total, _decimal(value, "sum term", nonnegative=True))
    return _float_upper(total)


def sum_products_upper(left, right):
    """Nonnegative dot product with outward multiplication and addition."""
    left, right = tuple(left), tuple(right)
    if len(left) != len(right):
        raise ValueError("Product sequences must have matching lengths")
    total = _Z
    for a, b in zip(left,right):
        a = _decimal(a,"product term",nonnegative=True)
        b = _decimal(b,"product term",nonnegative=True)
        total = _UP.add(total,_UP.multiply(a,b))
    return _float_upper(total)


def _add(a, b):
    return _DN.add(a[0], b[0]), _UP.add(a[1], b[1])


def _neg(a):
    # copy_negate never rounds, unlike unary minus under the global context.
    return a[1].copy_negate(), a[0].copy_negate()


def _mul(a, b):
    return (min(_DN.multiply(x, y) for x in a for y in b),
            max(_UP.multiply(x, y) for x in a for y in b))


def _divide_positive(a, divisor):
    return _DN.divide(a[0], divisor), _UP.divide(a[1], divisor)


def _max_abs(a):
    return max(a[0].copy_abs(), a[1].copy_abs())


def _remainder(x, order):
    # |x|**order/order!, all positive and rounded upward at every operation.
    result, magnitude = _ONE, x.copy_abs()
    for k in range(1, order+1):
        result = _UP.divide(_UP.multiply(result, magnitude), Decimal(k))
    return result


def _trig_exact(x):
    """Enclose sin/cos of this exact Decimal point using Taylor's theorem."""
    point = (x, x)
    negative_square = _neg(_mul(point, point))
    sin_term = sin_sum = point
    cos_term = cos_sum = _UNIT
    for n in range(1, TAYLOR_TERMS+1):
        sin_term = _divide_positive(_mul(sin_term, negative_square), Decimal(2*n*(2*n+1)))
        cos_term = _divide_positive(_mul(cos_term, negative_square), Decimal((2*n-1)*2*n))
        sin_sum, cos_sum = _add(sin_sum, sin_term), _add(cos_sum, cos_term)
    # Sin degree 257 also equals its degree-258 Taylor polynomial; cos degree
    # 256 equals degree 257. All real derivatives have absolute value <= 1.
    # This is a Lagrange remainder, not a monotonic alternating-series shortcut.
    sr, cr = _remainder(x, 2*TAYLOR_TERMS+3), _remainder(x, 2*TAYLOR_TERMS+2)
    return (_add(sin_sum, (sr.copy_negate(), sr)),
            _add(cos_sum, (cr.copy_negate(), cr)))


def _trig(interval):
    centre = _RN.divide(_RN.add(interval[0], interval[1]), _TWO)
    centre = min(interval[1], max(interval[0], centre))
    radius = max(_UP.subtract(centre, interval[0]), _UP.subtract(interval[1], centre))
    sin_box, cos_box = _trig_exact(centre)
    # Includes the preceding q+offset sum's rounding through the global
    # 1-Lipschitz property, instead of treating a rounded angle as exact.
    padding = (radius.copy_negate(), radius)
    return _add(sin_box, padding), _add(cos_box, padding)


@lru_cache(maxsize=16)
def _model(mdh):
    """Only model constants are cached; no observation or box is cached."""
    angles = tuple(_trig((row[2], row[2])) for row in mdh)
    radii = []
    for i, row in enumerate(mdh):
        radius = row[0].copy_abs()
        for later in mdh[i+1:]:
            radius = _UP.add(radius, later[0].copy_abs())
            radius = _UP.add(radius, later[1].copy_abs())
        radii.append(radius)
    return angles, tuple(radii)


def _identity():
    return [[_UNIT if i == j else _ZERO for j in range(4)] for i in range(3)]


def _compose(a, b):
    result = []
    for i in range(3):
        row = []
        for j in range(4):
            value = a[i][3] if j == 3 else _ZERO
            for k in range(3):
                value = _add(value, _mul(a[i][k], b[k][j]))
            row.append(value)
        result.append(row)
    return result


def _sqrt_upper(value):
    if not value:
        return _Z
    # Decimal.sqrt is correctly rounded to nearest, irrespective of directed
    # context rounding. Its next representable Decimal therefore encloses it.
    return _UP.next_plus(_RN.sqrt(value))


def _central_radii(mdh, centre, alpha_trig):
    suffix, result = _identity(), [_Z]*6
    for i in range(5, -1, -1):
        x, y = _max_abs(suffix[0][3]), _max_abs(suffix[1][3])
        result[i] = _sqrt_upper(_UP.add(_UP.multiply(x, x), _UP.multiply(y, y)))
        # In the frame after joint i's rotation, d_i is parallel to its axis;
        # the perpendicular radius is solely the downstream suffix's x/y.
        d, a, _, offset = mdh[i]
        sa, ca = alpha_trig[i]
        st, ct = _trig(_add((centre[i], centre[i]), (offset, offset)))
        link = [[ct, _neg(st), _ZERO, (a, a)],
                [_mul(ca, st), _mul(ca, ct), _neg(sa), _neg(_mul(sa, (d, d)))],
                [_mul(sa, st), _mul(sa, ct), ca, _mul(ca, (d, d))]]
        suffix = _compose(link, suffix)
    return result


def flange_box_bounds(mdh, origin_q, low, high):
    """Bound flange displacement from origin over every point of [low, high].

    For each joint, rho_i at the box centre is increased by the global
    downstream Lipschitz sum. This gives a uniform radius U_i throughout the
    box, not a single-pose Jacobian shortcut. A mathematical axis-at-a-time path
    inside the box proves the bound without assuming firmware interpolation.
    """
    if not isinstance(mdh, (list, tuple)) or len(mdh) != 6:
        raise ValueError("MDH must contain six (d,a,alpha,offset) rows")
    rows = []
    for row in mdh:
        if not isinstance(row, (list, tuple)) or len(row) != 4:
            raise ValueError("MDH must contain six (d,a,alpha,offset) rows")
        rows.append(tuple(_decimal(v, "MDH", maximum=10 if i < 2 else 32) for i,v in enumerate(row)))
    rows = tuple(rows)
    origin = _vector(origin_q, "origin", 32)
    lo, hi = _vector(low, "low", 32), _vector(high, "high", 32)
    if any(not a <= q <= b for a,q,b in zip(lo,origin,hi)):
        raise ValueError("Original joints must belong to the complete closed box")
    centre = [min(b,max(a,_RN.divide(_RN.add(a,b),_TWO))) for a,b in zip(lo,hi)]
    halfwidth = [max(_UP.subtract(c,a),_UP.subtract(b,c)) for a,b,c in zip(lo,hi,centre)]
    deltas = [(_DN.subtract(a,q),_UP.subtract(b,q)) for a,b,q in zip(lo,hi,origin)]
    excursions = [_max_abs(delta) for delta in deltas]
    alpha_trig, global_radii = _model(rows)
    central_radii = _central_radii(rows,centre,alpha_trig)
    radii = []
    for i, central in enumerate(central_radii):
        upper = central
        for k in range(i+1,6):
            upper = _UP.add(upper,_UP.multiply(global_radii[k],halfwidth[k]))
        radii.append(min(global_radii[i],upper))
    translation = _Z
    for radius, excursion in zip(radii,excursions):
        translation = _UP.add(translation,_UP.multiply(radius,excursion))
    parallel = rows[2][2] == _Z and rows[3][2] == _Z
    if parallel:
        cluster = _add(_add(deltas[1],deltas[2]),deltas[3])
        rotation_terms = [_max_abs(cluster),excursions[0],excursions[4],excursions[5]]
    else:
        rotation_terms = excursions
    rotation = _Z
    for value in rotation_terms:
        rotation = _UP.add(rotation,value)
    return {"translation_m":_float_upper(translation), "rotation_rad":_float_upper(rotation),
        "radius_bounds_m":[_float_upper(v) for v in radii],
        "global_radius_bounds_m":[_float_upper(v) for v in global_radii],
        "central_radius_upper_m":[_float_upper(v) for v in central_radii],
        "axis_excursions_rad":[_float_upper(v) for v in excursions],
        "parallel_axis_rotation_group":[2,3,4] if parallel else None,
        "method":"configuration_dependent_radius_and_"+("parallel_234_rotation" if parallel else "independent_rotation"),
        "numeric_method":"directed_decimal_intervals_with_lagrange_remainder",
        "decimal_precision":PRECISION,"taylor_terms":TAYLOR_TERMS,
        "includes_hold_budget":False,"includes_attachment_geometry":False,
        "hardware_commands_sent":0,"motion_permitted":False}
