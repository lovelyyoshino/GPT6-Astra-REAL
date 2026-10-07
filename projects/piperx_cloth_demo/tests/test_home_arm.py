"""Attended fixed home contract. Every CAN bus and physical socket is mocked."""
import copy
import math
import unittest
from unittest.mock import patch

from robot_tools import arms, home_arm, takeover
from test_backend import PROFILE
from test_execution import healthy_arm
from test_joint_recovery import RecoveryRobot, fk
from test_takeover import TakeoverFixture


SITE_Q = [0.656715, -0.0268955, 0.0622559, -0.021276, 0.351544, 0.012828]


class HomeRobot(RecoveryRobot):
    def __init__(self, side, clock, selected):
        super().__init__(side, clock)
        self.ctrl_mode = 1 if selected else 0
        self.motion.origin = SITE_Q[:]
        self.driver_enabled = [selected] * 6
        self.gripper_enabled = selected
        self.width = .00273 if selected else .01141


class HomeTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        for cfg in self.profile["arms"].values():
            cfg["model"] = "piper"
        self.stack.enter_context(patch.object(home_arm, "time", self.clock))
        self.make_robots()

    def make_robots(self, selected="right"):
        self.selected = selected
        self.robots = {s: HomeRobot(s, self.clock, s == selected) for s in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.hook = None

    def snapshot(self, robot, gripper):
        self.snapshot_count += 1
        q, motion = robot.motion.feedback()
        state = healthy_arm(self.clock.time())
        state["arm_status"].update(ctrl_mode=robot.ctrl_mode, teach_status=0,
                                  mode_feedback=robot.motion.mode, motion_status=motion)
        state["joints_rad"], state["pose_m_rad"] = q, fk(q)
        state["gripper"]["width_m"] = robot.width
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        for i, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None):
        return home_arm.home_arm(self.profile,
            journal or (lambda event, data: self.events.append((event, data))), self.selected)

    def check_contract(self, result, count=4):
        passive = "left" if self.selected == "right" else "right"
        self.assertEqual(result["hardware_commands_sent"], count)
        self.assertEqual(self.robots[passive].sent, [])
        self.assertEqual(result["passive_arm_commands_sent"], 0)
        self.assertEqual(result["gripper_target_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["task_motion_ready"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertFalse(result["path_collision_verified"])
        self.assertTrue(result["operator_full_path_review_required"])
        self.assertFalse(result["tracking_box_is_path_guarantee"])

    def failure(self, result, count=0):
        self.assertFalse(result["ok"], result)
        self.assertFalse(result["zero_target_observed"])
        self.check_contract(result, count)

    def test_site_pose_one_call_four_exact_frames_and_stable_windows(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.check_contract(result)
        self.assertTrue(result["zero_target_observed"])
        self.assertTrue(result["selected_arm_strictly_within_limits"])
        self.assertEqual(self.robots["right"].move_calls, 1)
        self.assertEqual([(f.arbitration_id, bytes(f.data)) for f in self.robots["right"].sent], list(home_arm.FRAMES))
        self.assertEqual([v["joint_index"] for v in result["initial_boundary_violations"]["right"]], [2, 3])
        for prefix in ("baseline", "stable"):
            self.assertGreaterEqual(result[prefix + "_duration_s"], 3)
            self.assertGreaterEqual(result[prefix + "_feedback_advances"], 20)
        self.assertEqual(result["original_enable_flags"]["left"], [False] * 7)
        self.assertEqual(result["original_enable_flags"]["right"], [True] * 7)
        self.assertFalse(result["joint_zero_calibrated"])

    def test_left_selection_and_passive_teach_mode_mixed_flags_preserved(self):
        self.make_robots("left")
        self.robots["right"].ctrl_mode = 2
        self.robots["right"].driver_enabled = [True, False, True, False, True, False]
        self.robots["right"].gripper_enabled = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.check_contract(result)

    def test_already_zero_does_not_switch_original_p_mode_and_waits(self):
        self.robots["right"].motion.origin = [0.] * 6
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.check_contract(result, 0)
        self.assertTrue(result["already_near_zero"])
        self.assertTrue(result["zero_target_observed"])
        self.assertFalse(result["move_j_mode_confirmed"])
        self.assertEqual(result["raw_mode_feedback"], 0)
        self.assertGreaterEqual(self.clock.elapsed, 6)
        self.assertEqual(self.robots["right"].move_calls, 0)

    def test_near_zero_nominal_violation_is_reported_without_command(self):
        self.robots["right"].motion.origin = [0., -.001, .001, 0., 0., 0.]
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.check_contract(result, 0)
        self.assertTrue(result["zero_target_observed"])
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertEqual([v["joint_index"] for v in result["final_boundary_violations"]["right"]], [2, 3])

    def test_fixed_api_rejects_target_and_missing_journal_or_arm(self):
        for invalid in (None, "both", True):
            with self.assertRaises(ValueError):
                home_arm.home_arm(self.profile, lambda event, data: None, invalid)
        with self.assertRaises(TypeError):
            home_arm.home_arm(self.profile, None, "right")
        with self.assertRaises(TypeError):
            home_arm.home_arm(self.profile, lambda event, data: None, "right", target=[0.] * 6)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_each_near_home_axis_bound_is_enforced(self):
        for i, bound in enumerate(home_arm.START_ABS_RAD):
            with self.subTest(axis=i + 1):
                self.make_robots()
                self.robots["right"].motion.origin[i] = bound + .00001
                self.failure(self.run_tool())

    def test_only_initial_j2_j3_five_degree_boundary_exception(self):
        for i, value in ((1, -math.pi / 36 - .00001), (2, math.pi / 36 + .00001)):
            with self.subTest(axis=i + 1):
                self.make_robots()
                self.robots["right"].motion.origin[i] = value
                self.failure(self.run_tool())

    def test_exact_exception_and_near_home_bounds_are_accepted(self):
        self.robots["right"].motion.origin = [math.pi / 4, -math.pi / 36, math.pi / 36,
                                               math.pi / 12, math.pi / 6, math.pi / 12]
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.check_contract(result)

    def test_selected_ctrl_driver_enable_and_teaching_rejected(self):
        for fault in ("ctrl", "driver", "unknown", "teach", "moving", "jaw_mode", "fault"):
            with self.subTest(fault=fault):
                self.make_robots()
                def hook(robot, state):
                    if robot.side != "right": return
                    if fault == "ctrl": state["arm_status"]["ctrl_mode"] = 0
                    elif fault == "driver": state["drivers"]["4"]["foc_status"]["driver_enable_status"] = False
                    elif fault == "unknown": state["gripper"]["foc_status"]["driver_enable_status"] = None
                    elif fault == "teach": state["arm_status"]["teach_status"] = 1
                    elif fault == "moving": state["arm_status"]["motion_status"] = 1
                    elif fault == "jaw_mode": state["gripper"]["mode"] = "angle"
                    elif fault == "fault": state["drivers"]["2"]["foc_status"]["collision_status"] = True
                self.hook = hook
                self.failure(self.run_tool())

    def test_baseline_drift_has_no_old_j4_exception(self):
        for field in ("j4", "xyz", "rotation", "jaw"):
            with self.subTest(field=field):
                self.make_robots()
                began = self.clock.elapsed
                def hook(robot, state):
                    if robot.side != "right" or self.clock.elapsed <= began + .1: return
                    if field == "j4":
                        state["joints_rad"][3] += .004
                        state["pose_m_rad"] = fk(state["joints_rad"])
                    elif field == "xyz": state["pose_m_rad"][0] += .0006
                    elif field == "rotation": state["pose_m_rad"][5] += .0031
                    else: state["gripper"]["width_m"] += .0006
                self.hook = hook
                self.failure(self.run_tool())

    def test_freshness_hundred_ms_and_complete_advancement_required(self):
        for fault in ("stale", "missing", "frozen"):
            with self.subTest(fault=fault):
                self.make_robots()
                initial = self.clock.time()
                def hook(robot, state):
                    stamps = state["fragment_timestamps_s"]
                    if fault == "stale":
                        for key in stamps: stamps[key] -= .11
                    elif fault == "missing": del stamps["gripper"]
                    else: stamps["gripper"] = initial
                self.hook = hook
                self.failure(self.run_tool())

    def test_freshness_limit_accepts_99ms_and_rejects_101ms(self):
        for age, accepted in ((.099, True), (.101, False)):
            with self.subTest(age=age):
                self.make_robots()
                for robot in self.robots.values():
                    robot.motion.origin = [0.0] * 6
                def hook(robot, state):
                    for key in state["fragment_timestamps_s"]:
                        state["fragment_timestamps_s"][key] -= age
                self.hook = hook
                result = self.run_tool()
                if accepted:
                    self.assertTrue(result["ok"], result)
                    self.assertAlmostEqual(result["max_checked_feedback_age_s"], age, places=5)
                    self.check_contract(result, 0)
                else:
                    self.failure(result, 0)
                    self.assertGreater(result["last_checked_feedback_age_s"], .1)

    def test_fk_feedback_processing_latency_is_included_in_freshness(self):
        original = self.robots["right"].fk
        def delayed_fk(q):
            self.clock.sleep(.11)
            return original(q)
        self.robots["right"].fk = delayed_fk
        result = self.run_tool()
        self.failure(result, 0)
        self.assertGreaterEqual(result["last_checked_feedback_age_s"], .1)

    def test_fk_feedback_position_or_rotation_disagreement_rejected(self):
        for index, delta in ((0, .0021), (5, .021)):
            with self.subTest(index=index):
                self.make_robots()
                def hook(robot, state):
                    if robot.side == "right": state["pose_m_rad"][index] += delta
                self.hook = hook
                self.failure(self.run_tool())

    def test_intent_journal_failure_or_new_baseline_drift_sends_nothing(self):
        for failure in ("journal", "state"):
            with self.subTest(failure=failure):
                self.make_robots()
                def journal(event, data):
                    if event == "home_intent":
                        if failure == "journal": raise OSError("disk full")
                        self.robots["right"].motion.origin[3] += .004
                self.failure(self.run_tool(journal))

    def test_each_frame_send_failure_and_swallowed_sdk_exception_abort_once(self):
        for index, (can_id, _) in enumerate(home_arm.FRAMES):
            for swallow in (False, True):
                with self.subTest(frame=hex(can_id), swallow=swallow):
                    self.make_robots()
                    self.robots["right"].fail_id = can_id
                    self.robots["right"].swallow_error = swallow
                    result = self.run_tool()
                    self.failure(result, index)
                    self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], index + 1)
                    self.assertEqual(self.robots["right"].move_calls, 1)

    def test_partial_sdk_sequence_is_not_completed_or_retried(self):
        self.robots["right"].partial = True
        result = self.run_tool()
        self.failure(result, 3)
        self.assertEqual(self.robots["right"].move_calls, 1)
        self.assertIn("Incomplete four-frame", result["errors"][0]["detail"])

    def test_duplicate_mode_frame_is_blocked(self):
        self.robots["right"].duplicate = True
        result = self.run_tool()
        self.failure(result, 1)
        self.assertTrue(result["guard_violations"])

    def test_wrong_id_bytes_and_can_flags_rejected_before_bus(self):
        for fault in ("id", "data", "fd", "extended", "remote"):
            with self.subTest(fault=fault):
                self.make_robots()
                def transform(frame):
                    if fault == "id": frame.arbitration_id = 0x159
                    elif fault == "data": frame.data[2] = 2
                    elif fault == "fd": frame.is_fd = True
                    elif fault == "extended": frame.is_extended_id = True
                    elif fault == "remote": frame.is_remote_frame = True
                    return frame
                self.robots["right"].frame_transform = transform
                result = self.run_tool()
                self.failure(result)
                self.assertTrue(result["guard_violations"])

    def test_passive_arm_and_both_gripper_sdk_senders_are_blocked(self):
        for target in ("passive", "left_jaw", "right_jaw"):
            with self.subTest(target=target):
                self.make_robots()
                def journal(event, data):
                    if event != "home_intent": return
                    robot = self.robots["right" if target == "right_jaw" else "left"]
                    frame = robot.can.Message(arbitration_id=0x159, data=bytes(8), is_extended_id=False)
                    (robot._send_msg if target == "passive" else robot.gripper._send_msg)(frame)
                result = self.run_tool(journal)
                self.failure(result)
                self.assertTrue(result["guard_violations"])

    def test_passive_direct_bus_attempt_blocked_even_during_active_ticket(self):
        original = self.robots["right"].move_j
        def attempted(q):
            robot = self.robots["left"]
            robot.comm.send_bus.send(robot.can.Message(arbitration_id=0x151,
                data=home_arm.FRAMES[0][1], is_extended_id=False))
            original(q)
        self.robots["right"].move_j = attempted
        result = self.run_tool()
        self.failure(result)
        self.assertTrue(result["guard_violations"])

    def test_mode_confirmation_must_not_regress(self):
        def hook(robot, state):
            if robot.side == "right" and robot.motion.started is not None:
                state["arm_status"]["mode_feedback"] = 1 if self.clock.elapsed - robot.motion.started < .05 else 0
        self.hook = hook
        result = self.run_tool()
        self.failure(result, 4)
        self.assertIn("movement mode", result["errors"][0]["detail"])

    def test_initial_old_mode_until_fresh_j_confirmation_is_allowed(self):
        def hook(robot, state):
            if (robot.side == "right" and robot.motion.started is not None
                    and self.clock.elapsed - robot.motion.started < .05):
                state["arm_status"]["mode_feedback"] = 0
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["move_j_mode_confirmed"])

    def test_initial_boundary_cannot_deepen_and_each_axis_tracking_box_enforced(self):
        for fault in ("deepen_j2", "deepen_j3", "box"):
            with self.subTest(fault=fault):
                self.make_robots()
                def hook(robot, state):
                    if robot.side != "right" or robot.motion.started is None: return
                    i, value = ((1, SITE_Q[1] - .0031) if fault == "deepen_j2" else
                                (2, SITE_Q[2] + .0031) if fault == "deepen_j3" else (0, -.0101))
                    state["joints_rad"][i] = value
                    state["pose_m_rad"] = fk(state["joints_rad"])
                self.hook = hook
                result = self.run_tool()
                self.failure(result, 4)
                self.assertIn("deepened" if fault != "box" else "tracking box", result["errors"][0]["detail"])

    def test_independent_joint_progress_does_not_require_synchronized_line(self):
        robot = self.robots["right"]
        def feedback():
            if robot.motion.started is None: return SITE_Q[:], 0
            elapsed = self.clock.elapsed - robot.motion.started
            q = [q0 * max(0., 1. - elapsed / (.1 + i * .02)) for i, q0 in enumerate(SITE_Q)]
            return q, int(any(abs(x) > 0 for x in q))
        robot.motion.feedback = feedback
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.check_contract(result)

    def test_passive_mode_flags_drift_and_both_jaw_states_preserved(self):
        for fault in ("mode", "enable", "passive_q", "passive_rotation", "left_jaw", "right_jaw", "jaw_enable"):
            with self.subTest(fault=fault):
                self.make_robots()
                def hook(robot, state):
                    if self.robots["right"].motion.started is None: return
                    if fault == "right_jaw" and robot.side == "right": state["gripper"]["width_m"] += .0006
                    if robot.side != "left": return
                    if fault == "mode": state["arm_status"]["ctrl_mode"] = 1
                    elif fault == "enable": state["drivers"]["1"]["foc_status"]["driver_enable_status"] = True
                    elif fault == "passive_q": state["joints_rad"][3] += .0031
                    elif fault == "passive_rotation": state["pose_m_rad"][5] += .0031
                    elif fault == "left_jaw": state["gripper"]["width_m"] += .0006
                    elif fault == "jaw_enable": state["gripper"]["foc_status"]["driver_enable_status"] = True
                self.hook = hook
                self.failure(self.run_tool(), 4)

    def test_every_fragment_must_advance_after_send(self):
        def hook(robot, state):
            started = self.robots["right"].motion.started
            if started is not None:
                state["fragment_timestamps_s"]["gripper"] = self.clock.time() - (self.clock.elapsed - started)
        self.hook = hook
        result = self.run_tool()
        self.failure(result, 4)
        detail = result["errors"][0]["detail"]
        # One frozen fragment can hit the unchanged 100 ms skew check before
        # the absolute-age check; either must reject without a second target.
        self.assertTrue("100 ms" in detail or "fragment_skew" in detail, detail)

    def test_no_motion_times_out_after_120_seconds_without_second_target(self):
        self.robots["right"].motion.accept = False
        result = self.run_tool()
        self.failure(result, 4)
        self.assertGreaterEqual(self.clock.elapsed, 123)
        self.assertIn("120 s", result["errors"][0]["detail"])
        self.assertEqual(self.robots["right"].move_calls, 1)

    def test_postsend_and_final_journal_failure_remove_success_claim(self):
        for fail_event in ("home_dispatched_unconfirmed", "home_zero_observed"):
            with self.subTest(event=fail_event):
                self.make_robots()
                def journal(event, data):
                    if event == fail_event: raise OSError("disk full")
                self.failure(self.run_tool(journal), 4)

    def test_cleanup_attempt_cannot_send_or_preserve_zero_claim(self):
        robot = self.robots["right"]
        robot.disconnect.side_effect = lambda: robot._send_msg(robot.can.Message(
            arbitration_id=0x151, is_extended_id=False, data=home_arm.FRAMES[0][1]))
        result = self.run_tool()
        self.failure(result, 4)
        self.assertTrue(result["guard_violations"])


if __name__ == "__main__": unittest.main()
