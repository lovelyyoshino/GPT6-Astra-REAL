"""Offline host-to-native-SDK first-target integration, with no seeded cache.

Only CAN, feedback and time are synthetic. Production planners, device methods,
host workers and durable ledger run unchanged. Fixture RGB/source geometry are
explicit test data and prove neither firmware trajectories nor physical safety.
"""
import copy
import json
import math
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import arms, linear_hold, pair_device, pair_initialization
from robot_tools import pair_joint_adapter, pair_preparation
from robot_tools import single_supervised_actions, supervised_actions, takeover
from robot_tools.joint_path import JointPathError
from robot_tools.pair_host import PairHost
from robot_tools.service import ToolService
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_joint_path import RAW, context as source_context
from test_pair_host import TASK
import test_host_joint_integration as joint_fixture
from test_joint_initialization import SAVED


class HostInitializationIntegrationTests(unittest.TestCase):
    observe = joint_fixture.HostJointIntegrationTests.observe
    request = joint_fixture.HostJointIntegrationTests.request
    submit = joint_fixture.HostJointIntegrationTests.submit

    def setUp(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.drivers.core import driver_context
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.status_type = ArmMsgFeedbackStatus
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.workspace = Path(self.directory.name)
        self.root = self.workspace/"projects/piperx_cloth_demo"
        (self.root/"configs").mkdir(parents=True)
        self.clock, self.frame = Clock(), 0
        self.sent, self.created, self.buses, self.bindings = [], [], [], {}
        self.profile = copy.deepcopy(PROFILE)
        self.profile["cameras"] = {"front":"synthetic-front", "left_wrist":"synthetic-left", "right_wrist":"synthetic-right"}
        for cfg in self.profile["arms"].values():
            cfg["model"] = "piper_x"
        self.channels = {side:cfg["channel"] for side,cfg in self.profile["arms"].items()}
        self.sides = {channel:side for side,channel in self.channels.items()}
        self.joints = {side:[math.radians(v/1000) for v in RAW] for side in self.channels}
        self.modes = dict.fromkeys(self.channels, 0)
        self.jaws = dict.fromkeys(self.channels, True)
        self.widths = dict.fromkeys(self.channels, .05)
        self.residual = [0.]*6
        self.fail_id = None
        self.thread_errors = []
        self.stack.enter_context(patch.object(threading,"excepthook",side_effect=self.thread_errors.append))
        self.addCleanup(self.check_async_errors)
        fixture = self
        class FakeCAN:
            def __init__(self, channel):
                self.channel = channel
                self.pending = RAW[:]
                fixture.buses.append(self)

            def recv(self, timeout=None):
                time.sleep(.001)
                return None

            def shutdown(self):
                pass

            def send(self, frame, timeout=None):
                if frame.arbitration_id == fixture.fail_id:
                    raise OSError("Synthetic transport failure")
                side = fixture.sides[self.channel]
                fixture.sent.append((side,copy.deepcopy(frame)))
                if frame.arbitration_id == 0x151:
                    fixture.modes[side] = frame.data[1]
                elif 0x155 <= frame.arbitration_id <= 0x157:
                    i = 2*(frame.arbitration_id-0x155)
                    self.pending[i:i+2] = struct.unpack(">ii",bytes(frame.data))
                    if frame.arbitration_id == 0x157:
                        fixture.joints[side] = [math.radians(v/1000)+r for v,r in zip(self.pending,fixture.residual)]
                elif frame.arbitration_id == 0x159:
                    fixture.widths[side] = struct.unpack(">i",bytes(frame.data[:4]))[0]/1e6
                    fixture.jaws[side] = True
                fixture.clock.sleep(.0001)

        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda:100.
            self.created.append(robot)
            self.bindings[id(robot)] = self.sides[config["comm"]["can"]["channel"]]
            return robot
        self.stack.enter_context(patch("socket.socket", side_effect=AssertionError("Hardware/network forbidden")))
        self.stack.enter_context(patch("robot_tools.cameras.capture_cameras", side_effect=AssertionError("Camera forbidden")))
        self.stack.enter_context(patch.object(arms,"_preflight"))
        for module in (arms,takeover,linear_hold,supervised_actions,single_supervised_actions,
                       pair_device,pair_preparation,pair_initialization,pair_joint_adapter):
            self.stack.enter_context(patch.object(module,"time",self.clock))
        self.stack.enter_context(patch.object(driver_context,"time",SimpleNamespace(
            monotonic=self.clock.monotonic,time=self.clock.time,sleep=time.sleep)))
        self.stack.enter_context(patch.object(can.interface,"Bus",side_effect=lambda **kw:FakeCAN(kw["channel"])))
        self.stack.enter_context(patch.object(sdk.AgxArmFactory,"create_arm",side_effect=create))
        self.stack.enter_context(patch.object(arms,"snapshot",side_effect=self.snapshot))
        (self.root/"configs/robot.json").write_text(json.dumps(self.profile))
        self.service = ToolService(self.root)

    def check_async_errors(self):
        # Native CANComm closes its connection on the injected send exception;
        # the concurrent native receive loop may then report this exact fault.
        # Audit it explicitly and fail the test for any unrelated thread error.
        for error in self.thread_errors:
            self.assertIsNotNone(self.fail_id)
            self.assertIs(error.exc_type,RuntimeError)
            self.assertEqual(str(error.exc_value),"CAN bus is not connected.")

    def snapshot(self, robot, gripper):
        self.clock.sleep(.001)
        side = self.bindings[id(robot)]
        state = healthy_arm(self.clock.time())
        state["arm_status"] = arms._plain(self.status_type(ctrl_mode=1,teach_status=0,
            mode_feedback=self.modes[side],motion_status=0,arm_status=0,err_code=0))
        state["joints_rad"] = self.joints[side][:]
        state["gripper"]["width_m"] = self.widths[side]
        state["gripper"]["foc_status"]["driver_enable_status"] = self.jaws[side]
        return state

    def sources(self, scene, arm):
        ctx = source_context(arm=arm)
        stable = Path(__file__).resolve().parents[1]/"data/piper_x_official"
        ctx["model_catalog"]["constants_path"] = str(stable/"sdk_constants.py")
        ctx["urdf_source"]["path"] = str(stable/"piper_x_description.urdf")
        ctx["geometry"]["available_clearance_m"] = .5  # Synthetic source, not scene measurement.
        return {key:copy.deepcopy(ctx[key]) for key in
                ("model_catalog","urdf_source","controller_limits","geometry")}

    def open(self, mode="ready"):
        self.host = PairHost(self.service.runs,self.profile,"first-target-integration",TASK,
            clock=self.clock.time,background=False,connection_mode=mode,joint_sources_provider=self.sources)
        self.service.pair_host,self.service.persistent = self.host,True
        self.addCleanup(self.host.close)
        self.host.open()
        self.device = self.host.device
        self.assertEqual(self.sent,[])
        self.assertEqual(len(self.created),2)
        self.assertEqual(len(self.buses),2)
        for side in self.channels:
            self.assertIsNone(self.device.joint_binding(side)["cached_target"])
            self.assertEqual(self.modes[side],0)

    def ids(self, side="right"):
        return [frame.arbitration_id for arm,frame in self.sent if arm==side]

    def initialize(self, arm="right", event="init-right"):
        scene = self.observe()
        response = self.host.initialize_joint_target(event,scene["observation_id"],arm,
            "Synthetic offline RGB semantics: both jaws empty and neither arm contacts an object")
        self.assertEqual(response["status"],"pending")
        return self.host.wait(event,10)

    def test_native_sdk_p_seed_then_normal_joint_uses_real_returned_cache(self):
        self.open()
        owner,deadline = self.host.owner,self.host.deadline
        initial = self.initialize()
        self.assertEqual(initial["status"],"completed",initial.get("receipt"))
        receipt = initial["receipt"]
        self.assertEqual(receipt["initialization_plan"]["purpose"],"seed_current")
        self.assertTrue(any(receipt["initialization_plan"]["target_raw"]))
        self.assertEqual(receipt["initialization_plan"]["cached_target_prior"],"unknown")
        self.assertEqual(receipt["hardware_commands_sent"],4)
        cache = self.device.joint_binding("right")["cached_target"]
        self.assertEqual(cache["event_id"],"init-right")
        self.assertEqual(cache["frame_receipts"],receipt["frame_receipts"])
        self.assertEqual(self.ids(),[0x151,0x155,0x156,0x157])
        request = self.request(event="after-real-init")  # A genuinely new fixture capture.
        self.assertEqual(self.submit(request)["status"],"pending")
        result = self.host.wait(request["event_id"],10)
        self.assertEqual(result["status"],"completed",result.get("receipt"))
        self.assertEqual(result["receipt"]["joint_path_plan"]["cached_target"],cache)
        self.assertEqual(result["receipt"]["hardware_commands_sent"],4)
        self.assertEqual(self.ids(),[0x151,0x155,0x156,0x157]*2)
        self.assertEqual(self.ids("left"),[])
        self.assertEqual((len(self.created),len(self.buses)),(2,2))
        self.assertEqual(self.host.owner,owner)
        self.assertEqual(self.host.deadline,deadline)
        self.assertEqual(self.host.ledger.peek_status()["steps"],2)
        self.assertIsNone(result["receipt"]["object_task_success"])
        self.assertIsNone(result["receipt"]["physical_stop_verified"])
        initial_observation = self.host.ledger.event("init-right")["payload"]["request"]["observation_id"]
        self.assertTrue(self.host.initialize_joint_target("init-right",initial_observation,
            "right","Synthetic offline RGB semantics: both jaws empty and neither arm contacts an object")["replayed"])
        self.assertEqual(len(self.sent),8)

    def test_historical_boundary_both_p_initialize_then_prepare_without_reconnect(self):
        history = json.loads(SAVED.read_text())["state"]["arms"]
        self.joints = {side:history[side]["joints_rad"][:] for side in self.channels}
        self.jaws = dict.fromkeys(self.channels,False)
        self.residual[1:3] = [-.001,.001]
        self.open("prepare")
        original = copy.deepcopy(self.device._preparation.anchor)
        owner,deadline = self.host.owner,self.host.deadline
        for side in ("left","right"):
            result = self.initialize(side,"init-"+side)
            self.assertEqual(result["status"],"completed",result.get("receipt"))
            receipt = result["receipt"]
            self.assertEqual(receipt["initialization_plan"]["purpose"],"startup_j2_j3")
            self.assertTrue(receipt["within_feedback_tolerance"])
            self.assertFalse(receipt["strict_nominal"])
            self.assertEqual(receipt["cached_target"]["target_raw"][1:3],[0,0])
            self.assertFalse(self.host.task_ready)
            self.assertFalse(any(self.jaws.values()))
        for side in self.channels:
            self.assertEqual(self.device._preparation.anchor[side]["gripper"],original[side]["gripper"])
            self.assertEqual(self.ids(side),[0x151,0x155,0x156,0x157])
            scene = self.observe()
            event = "prepare-"+side
            self.host.prepare_gripper(event,scene["observation_id"],side,
                "Synthetic offline scene shows selected empty jaw and finger clearance")
            prepared = self.host.wait(event,10)
            self.assertEqual(prepared["status"],"completed",prepared.get("receipt"))
        self.assertTrue(self.host.promote_ready()["task_ready"])
        self.assertEqual((len(self.created),len(self.buses)),(2,2))
        self.assertEqual(self.host.owner,owner)
        self.assertEqual(self.host.deadline,deadline)
        self.assertEqual(self.host.ledger.peek_status()["steps"],4)
        # Initialization now permits a source-bound inward ingress. This
        # request only changes J6 and still requests the observed out-of-limit
        # J2/J3 residual as a target, so it must remain inadmissible.
        request = self.request(event="residual-is-not-a-legal-target")
        with self.assertRaises(JointPathError) as caught:
            self.submit(request)
        self.assertEqual(caught.exception.code,"target_joint_limit")
        self.assertIsNone(self.host.ledger.event(request["event_id"]))
        self.assertEqual(self.host.ledger.peek_status()["steps"],4)
        self.assertFalse(self.host.fault_event.is_set())
        for side in self.channels:
            self.assertEqual(self.ids(side),[0x151,0x155,0x156,0x157,0x159])

    def test_partial_first_target_faults_ledger_and_never_creates_cache(self):
        self.open()
        self.fail_id = 0x156
        result = self.initialize()
        self.assertEqual(result["status"],"fault")
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertEqual(self.ids(),[0x151,0x155])
        self.assertEqual(self.ids("left"),[])
        self.assertEqual(self.host.ledger.peek_status()["steps"],1)


if __name__=="__main__":
    unittest.main()
