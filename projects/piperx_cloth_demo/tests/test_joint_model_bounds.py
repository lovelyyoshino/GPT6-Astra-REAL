"""Pure interval proof components plus independent SDK-FK counterexample search.

Samples can disprove a bound, not prove continuous robot motion or safety.
"""
import copy
from decimal import Decimal, localcontext
from fractions import Fraction
import itertools
import json
import math
import random
import unittest
from unittest.mock import patch

from robot_tools import arms, joint_model_bounds as bounds
from robot_tools.joint_path import _load_model
from robot_tools.model_compatibility import matrix_error, pose_matrix
from test_backend import PROFILE
from test_joint_path import context
from test_joint_initialization import SAVED


def tracking_box(origin, target, band=.003):
    return ([min(a,b)-band for a,b in zip(origin,target)],
            [max(a,b)+band for a,b in zip(origin,target)])


def reference_trig(x):
    """Independent 125-digit small-angle series followed by angle doubling.

    Its ~1e-120 numerical uncertainty is far smaller than tested module boxes.
    This is a numerical cross-check; module enclosure follows Taylor's theorem.
    """
    with localcontext() as ctx:
        ctx.prec=125
        y=x/Decimal(1024)
        sine,cosine,st,ct=y,Decimal(1),y,Decimal(1)
        for n in range(1,45):
            st *= -y*y/Decimal(2*n*(2*n+1))
            ct *= -y*y/Decimal((2*n-1)*2*n)
            sine+=st
            cosine+=ct
        for _ in range(10):
            sine,cosine=2*sine*cosine,cosine*cosine-sine*sine
        return sine,cosine


class JointModelBoundsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch("socket.socket",side_effect=AssertionError("No socket construction")):
            cls.sdk=arms._load_sdk(PROFILE["sdk_path"])
        from pyAgxArm.utiles.mdh_kinematics import fk_from_mdh, get_mdh
        cls.vendor_fk=staticmethod(fk_from_mdh)
        cls.vendor_mdh=staticmethod(get_mdh)
        source=context()
        cls.mdh=_load_model(source["model_catalog"],source["urdf_source"])[0]["mdh"]

    def setUp(self):
        self.addCleanup(patch.stopall)
        patch("socket.socket",side_effect=AssertionError("Hardware/network forbidden")).start()
        patch.object(self.sdk.AgxArmFactory,"create_arm",side_effect=AssertionError("Robot construction forbidden")).start()

    def vendor_error(self,mdh,origin,q):
        return matrix_error(pose_matrix(self.vendor_fk(mdh,origin)),pose_matrix(self.vendor_fk(mdh,q)))

    def test_zero_box_has_zero_motion_and_does_not_modify_inputs(self):
        origin=[.2,.3,-.4,.1,.2,.1]
        data=copy.deepcopy((self.mdh,origin))
        result=bounds.flange_box_bounds(self.mdh,origin,origin,origin)
        self.assertEqual(result["translation_m"],0.)
        self.assertEqual(result["rotation_rad"],0.)
        self.assertFalse(result["includes_hold_budget"])
        self.assertFalse(result["includes_attachment_geometry"])
        self.assertFalse(result["motion_permitted"])
        self.assertEqual(result["hardware_commands_sent"],0)
        self.assertEqual((self.mdh,origin),data)

    def test_parallel_orientation_reduction_keeps_asynchronous_combinations(self):
        origin=[.2,0.,0.,.1,.2,.1]
        target=origin[:]
        target[1:3]=[.010,-.010]
        low,high=tracking_box(origin,target)
        result=bounds.flange_box_bounds(self.mdh,origin,low,high)
        self.assertEqual(result["parallel_axis_rotation_group"],[2,3,4])
        self.assertAlmostEqual(result["rotation_rad"],.028,places=14)
        self.assertLess(result["rotation_rad"],sum(result["axis_excursions_rad"]))
        self.assertLess(bounds.sum_upper([result["rotation_rad"],bounds.sum_upper([.003]*6)]),.05)
        for count in range(4):
            # Actual targets arrive in pairs 155/156/157; J2 and J3 may be mixed.
            mixed=target[:count*2]+origin[count*2:]
            error=self.vendor_error(self.mdh,origin,mixed)
            self.assertLessEqual(error["position_error_m"],result["translation_m"])
            self.assertLessEqual(error["so3_error_rad"],result["rotation_rad"])
        j2_only=origin[:]
        j2_only[1]=.010
        self.assertGreater(self.vendor_error(self.mdh,origin,j2_only)["so3_error_rad"],.009)

    def test_other_model_and_even_tiny_nonparallel_twist_use_triangle_fallback(self):
        origin=[.2,.3,-.4,.1,.2,.1]
        low,high=tracking_box(origin,[q+.002 for q in origin])
        for mdh in (list(self.vendor_mdh("piper")),copy.deepcopy(self.mdh)):
            if mdh==self.mdh:
                mdh=[list(row) for row in mdh]
                mdh[2][2]=1e-12
            result=bounds.flange_box_bounds(mdh,origin,low,high)
            self.assertIsNone(result["parallel_axis_rotation_group"])
            self.assertIn("independent_rotation",result["method"])
            self.assertAlmostEqual(result["rotation_rad"],.030,places=14)

    def test_last_axis_zero_flange_radius_does_not_describe_attachments(self):
        origin=[.2,.3,-.4,.1,.2,.1]
        low,high=origin[:],origin[:]
        low[5]-=.02
        high[5]+=.02
        result=bounds.flange_box_bounds(self.mdh,origin,low,high)
        self.assertEqual(result["radius_bounds_m"][5],0.)
        self.assertGreater(result["global_radius_bounds_m"][5],0.)
        self.assertEqual(result["translation_m"],0.)
        self.assertFalse(result["includes_attachment_geometry"])
        self.assertGreater(self.vendor_error(self.mdh,origin,high)["so3_error_rad"],.019)

    def test_adversarial_folded_chain_requires_uniform_downstream_padding(self):
        mdh=[[0.,0.,0.,0.] for _ in range(6)]
        mdh[1][2]=math.pi/2
        mdh[2][1]=mdh[3][1]=1.
        origin=[0.,0.,math.pi,0.,0.,0.]
        low,high=origin[:],origin[:]
        low[1]-=.2
        high[1]+=.2
        low[2]-=.2
        high[2]+=.2
        result=bounds.flange_box_bounds(mdh,origin,low,high)
        self.assertLess(result["central_radius_upper_m"][1],1e-12)
        self.assertGreaterEqual(result["radius_bounds_m"][1],2*math.sin(.1))
        self.assertGreater(result["radius_bounds_m"][1],.19)
        for q2,q3 in itertools.product((low[1],high[1]),(low[2],high[2])):
            point=origin[:]
            point[1:3]=[q2,q3]
            error=self.vendor_error(mdh,origin,point)
            self.assertLessEqual(error["position_error_m"],result["translation_m"])

    def test_saved_both_arm_tail_boxes_cover_all_corners_and_random_independent_points(self):
        states=json.loads(SAVED.read_text())["state"]["arms"]
        rng=random.Random(346971)
        for side in ("left","right"):
            origin=states[side]["joints_rad"][:]
            origin[1:3]=[-.001,.001]  # Synthetic admitted tail; no eligibility claim.
            target=origin[:]
            target[1:3]=[.01001,-.01001]
            low,high=tracking_box(origin,target)
            result=bounds.flange_box_bounds(self.mdh,origin,low,high)
            self.assertTrue(all(u<=g for u,g in zip(result["radius_bounds_m"],result["global_radius_bounds_m"])))
            samples=list(itertools.product(*zip(low,high)))
            samples += [[rng.uniform(a,b) for a,b in zip(low,high)] for _ in range(128)]
            for q in samples:
                error=self.vendor_error(self.mdh,origin,q)
                self.assertLessEqual(error["position_error_m"],result["translation_m"])
                self.assertLessEqual(error["so3_error_rad"],result["rotation_rad"])

    def test_random_broad_poses_and_nonparallel_models_against_vendor_fk(self):
        rng=random.Random(9251)
        for model in ("piper_x","piper"):
            mdh=list(self.vendor_mdh(model))
            for _ in range(4):
                origin=[rng.uniform(-2.,2.) for _ in range(6)]
                low=[q-rng.uniform(.001,.3) for q in origin]
                high=[q+rng.uniform(.001,.3) for q in origin]
                result=bounds.flange_box_bounds(mdh,origin,low,high)
                for _ in range(40):
                    point=[rng.uniform(a,b) for a,b in zip(low,high)]
                    error=self.vendor_error(mdh,origin,point)
                    self.assertLessEqual(error["position_error_m"],result["translation_m"])
                    self.assertLessEqual(error["so3_error_rad"],result["rotation_rad"])

    def test_decimal_trig_encloses_independent_reduced_angle_reference_at_full_range(self):
        for x in (Decimal(0),Decimal("1e-60"),Decimal(1),Decimal(-32),Decimal("63.999999999"),Decimal(64)):
            intervals=bounds._trig_exact(x)
            reference=reference_trig(x)
            for interval,point in zip(intervals,reference):
                self.assertLessEqual(interval[0],point)
                self.assertGreaterEqual(interval[1],point)
                self.assertLess(interval[1]-interval[0],Decimal("1e-35"))

    def test_global_decimal_rounding_cannot_shrink_result(self):
        origin=[.2,0.,0.,.1,.2,.1]
        low,high=tracking_box(origin,[q+.001 for q in origin])
        baseline=bounds.flange_box_bounds(self.mdh,origin,low,high)
        with localcontext() as ctx:
            ctx.prec=5
            actual=bounds.flange_box_bounds(self.mdh,origin,low,high)
        self.assertEqual(actual,baseline)

    def test_outward_sum_and_products_preserve_sub_ulp_threshold_excess(self):
        tiny=2.**-65
        for threshold in (.020,.05):
            self.assertEqual(threshold+tiny,threshold)
            result=bounds.sum_upper([threshold,tiny])
            self.assertGreater(result,threshold)
            self.assertGreaterEqual(Fraction.from_float(result),Fraction.from_float(threshold)+Fraction.from_float(tiny))
        rng=random.Random(812)
        for _ in range(64):
            a,b=[rng.random() for _ in range(6)],[rng.random() for _ in range(6)]
            exact=sum(Fraction.from_float(x)*Fraction.from_float(y) for x,y in zip(a,b))
            self.assertGreaterEqual(Fraction.from_float(bounds.sum_products_upper(a,b)),exact)
        smallest=float.fromhex("0x0.0000000000001p-1022")
        self.assertEqual(bounds.sum_products_upper([smallest],[smallest]),smallest)
        self.assertEqual(bounds.sum_upper([]),0.)
        self.assertEqual(bounds.sum_products_upper([],[]),0.)

    def test_invalid_shapes_ranges_and_inverted_or_unanchored_box_reject(self):
        zero=[0.]*6
        for args in (([],zero,zero,zero),(self.mdh,zero,[1.]*6,[2.]*6),
                     (self.mdh,zero,[0.]*6,[-1.]*6),(self.mdh,[True]*6,zero,zero),
                     (self.mdh,zero,[-33.]*6,zero),(self.mdh,zero,zero,[float("nan")]*6)):
            with self.assertRaises(ValueError):
                bounds.flange_box_bounds(*args)
        for values in ([True],[float("nan")],[float("inf")],[-1.],[1e308,1e308]):
            with self.assertRaises(ValueError):
                bounds.sum_upper(values)
        with self.assertRaises(ValueError):
            bounds.sum_products_upper([1.],[1.,2.])
        with self.assertRaises(ValueError):
            bounds.sum_products_upper([1.],[-1.])


if __name__=="__main__":
    unittest.main()
