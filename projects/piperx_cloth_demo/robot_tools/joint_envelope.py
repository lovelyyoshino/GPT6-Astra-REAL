"""Pure MDH tracking-box bound shared by supervised joint operations.

The caller supplies a physical attachment bound. This does not bound cached or
partially received targets and does not establish actual obstacle clearance.
"""
import math


def tracking_sweep(model, origin, target, attachment_radius_m, tolerances,
                   link_body_allowance_m):
    from pyAgxArm.utiles.mdh_kinematics import get_mdh
    mdh = get_mdh(model)
    if len(mdh) != 6 or any(not math.isfinite(v) for row in mdh for v in row):
        raise RuntimeError("A finite six-axis manufacturer MDH chain is required")
    # Modified DH places a_i before joint i's rotation: retain d_i and every
    # subsequent translation as a conservative radius for every moving point.
    radii = [abs(row[0]) + sum(abs(link[0])+abs(link[1]) for link in mdh[i+1:])
             + attachment_radius_m + link_body_allowance_m for i, row in enumerate(mdh)]
    excursions = [abs(end-start)+tolerance
                  for start, end, tolerance in zip(origin, target, tolerances)]
    return {"sweep_bound_m": sum(radius*delta for radius, delta in zip(radii, excursions)),
            "sweep_axis_radii_m": radii, "sweep_axis_excursions_rad": excursions,
            "sweep_reference_joints_rad": list(origin),
            "mdh_source": "Manufacturer get_mdh(model), modified DH; sum of absolute remaining translations"}
