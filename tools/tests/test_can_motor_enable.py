"""Startup routing tests; sockets are blocked and service calls are faked."""

from contextlib import redirect_stdout
from copy import deepcopy
from enum import IntEnum
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "projects/piperx_cloth_demo"))
import can_motor_enable as helper
from robot_tools import arms


def arm(enabled=False):
    return {"status": "complete", "timestamp": 1000.0,
            "fragment_timestamps_s": dict.fromkeys(arms.PARTS + arms.DRIVERS + ("gripper",), 1000.0),
            "pose_m_rad": [0.0] * 6, "joints_rad": [0.0] * 6,
            "arm_status": {"ctrl_mode": int(enabled), "arm_status": 0, "motion_status": 0,
                           "err_code": 0, "err_status": dict.fromkeys(arms.ARM_ERRORS, False)},
            "drivers": {str(i): {"foc_status": {**dict.fromkeys(arms.DRIVER_ERRORS, False),
                                                 "driver_enable_status": enabled}} for i in range(1, 7)},
            "gripper": {"mode": "width", "width_m": 0.04, "force_N": 0.0,
                        "foc_status": {**dict.fromkeys(arms.GRIPPER_ERRORS, False), "driver_enable_status": False}}}


class MotorStartupTests(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        for context in (redirect_stdout(self.output),
                        patch("robot_tools.startup_reset.inspect", return_value=None),
                        patch("socket.socket", side_effect=AssertionError("No hardware sockets"))):
            context.__enter__()
            self.addCleanup(context.__exit__, None, None, None)
        self.before = {side: arm() for side in helper.SIDES}
        self.after = {side: arm(True) for side in helper.SIDES}
        self.startup_ok = True
        self.service = Mock()
        self.service.call.side_effect = self.call
        self.ownership = Mock()

    def call(self, name, arguments):
        if name == "robot_read_state":
            return {"ok": True, "state": {"status": "complete", "arms": deepcopy(self.before)}}
        self.assertIn(name, ("robot_startup_arm", "robot_startup_arms"))
        return {"ok": self.startup_ok, "after": deepcopy(self.after),
                "record_path": "fake-receipt.json", "errors": [] if self.startup_ok else ["uncertain send"]}

    def execute(self, selected=("left", "right")):
        helper.execute(self.service, {"selected_arms": list(selected)}, self.ownership)

    def test_both_disabled_call_existing_pair_startup_once(self):
        self.execute()
        self.assertEqual(self.service.call.call_count, 2)
        self.service.call.assert_called_with("robot_startup_arms", {})
        self.assertEqual(self.output.getvalue().count("（6/6）"), 2)

    def test_single_selection_calls_only_named_arm(self):
        self.execute(("right",))
        self.service.call.assert_called_with("robot_startup_arm", {"arm": "right"})

    def test_one_already_enabled_routes_only_missing_arm(self):
        self.before["left"] = arm(True)
        self.execute()
        self.service.call.assert_called_with("robot_startup_arm", {"arm": "right"})

    def test_already_enabled_is_receive_only_and_accepts_sdk_intenum(self):
        class Mode(IntEnum):
            CAN = 1
        self.before = {side: arm(True) for side in helper.SIDES}
        for state in self.before.values():
            state["arm_status"]["ctrl_mode"] = Mode.CAN
        self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})
        self.ownership.assert_not_called()

    def test_previous_startup_failure_does_not_block_enabled_noop(self):
        self.before = {side: arm(True) for side in helper.SIDES}
        self.ownership.side_effect = RuntimeError("本次电脑启动已有未决或失败的使能事务，禁止重复发送。")
        self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})
        self.ownership.assert_not_called()
        self.assertIn("本次仅查看反馈，未发送使能命令", self.output.getvalue())
        self.assertIn("原有故障记录保留", self.output.getvalue())

    def test_previous_startup_failure_still_blocks_any_needed_enable(self):
        for already_enabled in (False, True):
            with self.subTest(one_already_enabled=already_enabled):
                self.before["left"] = arm(already_enabled)
                self.service.reset_mock()
                self.ownership.side_effect = RuntimeError("未决或失败的使能事务")
                with self.assertRaisesRegex(RuntimeError, "未决或失败"):
                    self.execute()
                self.service.call.assert_called_once_with("robot_read_state", {})

    def test_already_enabled_with_stale_feedback_still_fails(self):
        self.before = {side: arm(True) for side in helper.SIDES}
        self.before["right"]["fragment_timestamps_s"]["driver_state_6"] = 990.0
        with self.assertRaisesRegex(RuntimeError, "反馈不健康"):
            self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_partial_startup_refuses_without_replay(self):
        self.before["right"]["drivers"]["2"]["foc_status"]["driver_enable_status"] = True
        with self.assertRaisesRegex(RuntimeError, "不重放部分启动"):
            self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_can_mode_but_disabled_is_not_retried(self):
        self.before["left"]["arm_status"]["ctrl_mode"] = 1
        with self.assertRaises(RuntimeError):
            self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_faulty_peer_prevents_single_arm_dispatch(self):
        self.before["left"]["arm_status"]["err_code"] = 1
        with self.assertRaisesRegex(RuntimeError, "反馈不健康"):
            self.execute(("right",))
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_stale_feedback_prevents_dispatch(self):
        self.before["left"]["fragment_timestamps_s"]["arm_status"] = 990
        with self.assertRaisesRegex(RuntimeError, "反馈不健康"):
            self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_uncertain_startup_result_preserved_without_retry(self):
        self.startup_ok = False
        with self.assertRaisesRegex(RuntimeError, "uncertain send"):
            self.execute()
        self.assertEqual(self.service.call.call_count, 2)
        self.assertNotIn("（6/6）", self.output.getvalue())

    def test_post_enable_failure_reports_observed_flags_without_claiming_success(self):
        result = {"ok": False, "status": "aborted_after_dispatch", "after": self.after,
                  "errors": [{"type": "RuntimeError", "detail": "right drift joint_rad=0.005603 exceeds 0.003000"}],
                  "drift": {"right": {"joint_rad": 0.005602506898901798}},
                  "record_path": "fake-receipt.json", "hold_not_validated": True}
        original = deepcopy(result)
        self.service.call.side_effect = [
            {"ok": True, "state": {"status": "complete", "arms": self.before}}, result]
        with self.assertRaises(RuntimeError) as raised:
            self.execute()
        message = str(raised.exception)
        self.assertEqual(message.count("末次回执关节使能=6/6"), 2)
        self.assertIn("aborted_after_dispatch", message)
        self.assertIn("right drift joint_rad=0.005603 exceeds 0.003000", message)
        self.assertIn("不要求回原点", message)
        self.assertIn("使能位不代表位置保持已验证", message)
        self.assertIn("fake-receipt.json", message)
        self.assertNotIn("fragment_timestamps_s", message)
        self.assertEqual(result, original)
        self.assertEqual(self.service.call.call_count, 2)

    def test_failed_startup_with_partial_or_missing_feedback_is_not_all_enabled(self):
        self.after["left"]["drivers"]["6"]["foc_status"]["driver_enable_status"] = False
        self.after["right"]["drivers"]["6"]["foc_status"].pop("driver_enable_status")
        message = helper.startup_failure_summary({"after": self.after}, helper.SIDES)
        self.assertIn("左臂：末次回执关节使能=5/6", message)
        self.assertIn("右臂：末次回执不完整", message)
        self.assertNotIn("6/6", message)

    def test_stale_or_incomplete_after_state_does_not_report_confirmed_enable(self):
        for status in ("stale", "incomplete"):
            with self.subTest(status=status):
                self.after["right"]["status"] = status
                message = helper.startup_failure_summary({"after": self.after}, ["right"])
                self.assertIn("关节使能状态未确认", message)
                self.assertNotIn("6/6", message)

    def test_nonzero_initial_pose_is_forwarded_to_existing_startup(self):
        for state in self.before.values():
            state["joints_rad"] = [0.7, -0.03, 0.05, 0.0, 0.4, -0.11]
            state["pose_m_rad"] = [0.04, 0.03, 0.16, -2.8, 1.2, -2.1]
        self.execute()
        self.assertEqual(self.service.call.call_args_list, [
            unittest.mock.call("robot_read_state", {}), unittest.mock.call("robot_startup_arms", {})])
        self.assertIn("无需回原点", self.output.getvalue())

    def test_ok_without_all_enable_feedback_cannot_claim_success(self):
        self.after["right"]["drivers"]["6"]["foc_status"]["driver_enable_status"] = False
        with self.assertRaisesRegex(RuntimeError, "缺少完整"):
            self.execute(("right",))
        self.assertEqual(self.service.call.call_count, 2)
        self.assertNotIn("（6/6）", self.output.getvalue())

    def test_owner_appearing_after_read_prevents_startup(self):
        self.ownership.side_effect = RuntimeError("host appeared")
        with self.assertRaisesRegex(RuntimeError, "host appeared"):
            self.execute()
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_main_and_binding_check_reach_enabled_noop_despite_failed_claim(self):
        self.before = {side: arm(True) for side in helper.SIDES}
        with patch.object(helper, "check_bindings") as bindings, \
                patch.object(helper, "check_ownership", side_effect=RuntimeError("failed claim")) as owner, \
                patch("robot_tools.service.ToolService", return_value=self.service) as factory:
            self.assertEqual(helper.main(["--check", "can0", "can1"]), 0)
            factory.assert_not_called()
            owner.assert_not_called()
            self.assertEqual(helper.main(["can0", "can1"]), 0)
            self.assertEqual(bindings.call_count, 2)
            self.service.call.assert_called_once_with("robot_read_state", {})
            owner.assert_not_called()

    def test_network_startup_check_still_refuses_failed_claim_without_reading_devices(self):
        with patch.object(helper, "check_bindings"), \
                patch.object(helper, "check_ownership", side_effect=RuntimeError("failed claim")), \
                patch("robot_tools.service.ToolService") as factory:
            with self.assertRaisesRegex(RuntimeError, "failed claim"):
                helper.main(["--check-startup", "can0", "can1"])
            factory.assert_not_called()

    def test_existing_ledger_fault_is_not_cleared(self):
        with patch("robot_tools.pair_ledger.platform_state", return_value={
                "owner": None, "fault": {"reason": "old fault"}, "pending_events": []}), \
                patch("robot_tools.arm_power_cycle.has_startup_history", return_value=False), \
                patch("robot_tools.reboot_startup.inspect", side_effect=RuntimeError("current-boot fault")):
            with self.assertRaisesRegex(RuntimeError, "current-boot fault"):
                helper.check_ownership()

    def test_archived_fault_routes_through_explicit_reboot_tool(self):
        from robot_tools.reboot_startup import CONFIRMATION
        self.ownership.return_value = {"route": "reboot_startup"}
        self.service.call.side_effect = [
            {"ok": True, "state": {"status": "complete", "arms": self.before}},
            {"ok": True, "after": self.after}]
        confirm = Mock(return_value=CONFIRMATION)
        helper.execute(self.service, {"selected_arms": ["left", "right"]}, self.ownership, confirm)
        confirm.assert_called_once_with()
        self.service.call.assert_called_with("robot_startup_after_host_reboot", {
            "arm": "both", "power_cycle_and_clearance_statement": CONFIRMATION})

    def test_same_host_arm_cycle_routes_only_disabled_left(self):
        from robot_tools.arm_power_cycle import confirmation
        self.before["right"] = arm(True)
        self.ownership.return_value = {"route": "arm_power_cycle_startup"}
        self.service.call.side_effect = [
            {"ok": True, "state": {"status": "complete", "arms": self.before}},
            {"ok": True, "after": self.after}]
        confirm = Mock(return_value=("cycle-1", confirmation("left")))
        helper.execute(self.service, {"selected_arms": ["left", "right"]}, self.ownership,
                       arm_cycle_confirmation=confirm)
        confirm.assert_called_once_with("left")
        self.service.call.assert_called_with("robot_startup_after_arm_power_cycle", {
            "arm": "left", "power_cycle_id": "cycle-1",
            "power_cycle_and_clearance_statement": confirmation("left")})

    def test_only_typed_host_boot_refusal_selects_new_arm_cycle_route(self):
        from robot_tools.reboot_startup import HostBootRequired
        with patch("robot_tools.pair_ledger.platform_state", return_value={
                "owner": "old", "fault": {}, "pending_events": []}), \
                patch("robot_tools.arm_power_cycle.has_startup_history", return_value=False), \
                patch("robot_tools.reboot_startup.inspect", side_effect=HostBootRequired("same boot")), \
                patch("robot_tools.arm_power_cycle.inspect", return_value={"route": "arm_power_cycle_startup"}) as inspect:
            self.assertEqual(helper.check_ownership(), {"route": "arm_power_cycle_startup"})
            inspect.assert_called_once()

    def test_completed_host_startup_then_arm_reset_uses_new_cycle_even_without_task(self):
        with patch("robot_tools.pair_ledger.platform_state", return_value={
                "owner": "old", "fault": {}, "pending_events": []}), \
                patch("robot_tools.arm_power_cycle.has_startup_history", return_value=True), \
                patch("robot_tools.reboot_startup.inspect") as old, \
                patch("robot_tools.arm_power_cycle.inspect", return_value={"route": "arm_power_cycle_startup"}) as new:
            self.assertEqual(helper.check_ownership(), {"route": "arm_power_cycle_startup"})
            old.assert_not_called()
            new.assert_called_once()

    def test_cycle_confirmation_rejection_sends_nothing(self):
        self.ownership.return_value = {"route": "arm_power_cycle_startup"}
        confirm = Mock(side_effect=RuntimeError("not confirmed"))
        with self.assertRaisesRegex(RuntimeError, "not confirmed"):
            helper.execute(self.service, {"selected_arms": ["left"]}, self.ownership,
                           arm_cycle_confirmation=confirm)
        self.service.call.assert_called_once_with("robot_read_state", {})

    def test_software_reset_routes_through_new_latch_with_explicit_scene(self):
        from robot_tools.reboot_startup import CONFIRMATION
        self.ownership.return_value = {"route": "state_reset_startup", "status": "awaiting_startup"}
        self.service.call.side_effect = [
            {"ok": True, "state": {"status": "complete", "arms": self.before}},
            {"ok": True, "after": self.after}]
        confirm = Mock(return_value=CONFIRMATION)
        helper.execute(self.service, {"selected_arms": ["left", "right"]}, self.ownership,
                       reset_confirmation=confirm)
        self.service.call.assert_called_with("robot_startup_after_state_reset", {
            "arm": "both", "power_cycle_and_clearance_statement": CONFIRMATION})

    def test_reset_main_never_constructs_a_device_service(self):
        with patch.object(helper, "check_bindings"), \
                patch("robot_tools.startup_reset.reset", return_value={"reset_performed": True}) as reset, \
                patch("robot_tools.service.ToolService") as factory:
            self.assertEqual(helper.main(["--reset-state", "can0", "can1"]), 0)
            reset.assert_called_once_with(helper.PROJECT)
            factory.assert_not_called()

    def test_arm_cycle_prompt_names_only_selected_arm_and_new_id(self):
        from robot_tools.arm_power_cycle import confirmation
        with patch.object(helper.sys.stdin, "isatty", return_value=True), \
                patch("builtins.input", return_value="yes") as prompt:
            first, statement = helper.confirm_arm_cycle_scene("left")
            second, _ = helper.confirm_arm_cycle_scene("left")
        self.assertNotEqual(first, second)
        self.assertEqual(statement, confirmation("left"))
        self.assertIn("左臂重新断电再上电", prompt.call_args[0][0])

    def test_arm_cycle_noninteractive_never_auto_confirms(self):
        with patch.object(helper.sys.stdin, "isatty", return_value=False), \
                self.assertRaisesRegex(RuntimeError, "现场确认"):
            helper.confirm_arm_cycle_scene("left")

    def test_plan_uses_configured_mapping_and_rejects_unknown_bus(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "configs").mkdir()
            (root / "configs/robot.json").write_text(json.dumps({"arms": {
                "left": {"channel": "can9"}, "right": {"channel": "can5"}}}))
            self.assertEqual(helper.plan(["can5"], root)["selected_arms"], ["right"])
            with self.assertRaisesRegex(RuntimeError, "没有机械臂绑定"):
                helper.plan(["can0"], root)


if __name__ == "__main__":
    unittest.main()
