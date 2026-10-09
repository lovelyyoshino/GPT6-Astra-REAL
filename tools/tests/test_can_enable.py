"""Offline CAN setup checks: every subprocess is mocked, no device writes."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import can_enable


def link(name="can0", up=False, bitrate=None, state="STOPPED", index=4):
    return {"ifname": name, "ifindex": index, "flags": ["UP"] if up else [],
            "linkinfo": {"info_kind": "can", "info_data": {
                "state": state, "bittiming": {"bitrate": bitrate}}}}


class CanEnableTests(unittest.TestCase):
    def setUp(self):
        self.links = {"can0": link(), "can1": link("can1", index=5)}
        self.commands = []
        self.fail_up = False
        self.disconnect = False
        self.fail_verification = False
        self.motor_calls = []
        self.motor_preflight_error = False
        self.binding_error = False
        self.output = io.StringIO()
        for context in (redirect_stdout(self.output),
                        patch.object(can_enable.subprocess, "run", side_effect=self.run_command),
                        patch.object(can_enable.shutil, "which", side_effect=lambda name: "/usr/bin/" + name),
                        patch.object(can_enable.os, "geteuid", return_value=1000)):
            context.__enter__()
            self.addCleanup(context.__exit__, None, None, None)

    def run_command(self, command, **kwargs):
        self.commands.append(command)
        if any(Path(part).name == "can_motor_enable.py" for part in command):
            if any(flag in command for flag in ("--check", "--plan", "--check-startup", "--reset-state")):
                if self.binding_error:
                    return subprocess.CompletedProcess(command, 1, "", "USB binding mismatch")
                if self.motor_preflight_error and "--check-startup" in command:
                    return subprocess.CompletedProcess(command, 1, "", "persistent owner/fault")
                return subprocess.CompletedProcess(command, 0, json.dumps({"required_interfaces": ["can0", "can1"]}), "")
            self.assertTrue(all("UP" in item["flags"] for item in self.links.values()))
            self.motor_calls.append(command)
            return subprocess.CompletedProcess(command, 0)
        if command == ["/usr/bin/sudo", "-v"]:
            if self.disconnect:
                self.links["can0"]["ifindex"] += 100
            return subprocess.CompletedProcess(command, 0)
        if command[1:] == ["-j", "-details", "link", "show"]:
            return subprocess.CompletedProcess(command, 0, json.dumps(list(self.links.values())), "")
        self.assertEqual(command[:6], ["/usr/bin/sudo", "-n", "/usr/bin/ip", "link", "set", "dev"])
        item = self.links[command[6]]
        if command[7:] == ["up"]:
            if self.fail_up:
                return subprocess.CompletedProcess(command, 2)
            item["flags"] = ["UP"]
            item["linkinfo"]["info_data"]["state"] = "BUS-OFF" if self.fail_verification else "ERROR-ACTIVE"
        else:
            self.assertEqual(command[7:10], ["type", "can", "bitrate"])
            item["linkinfo"]["info_data"]["bittiming"]["bitrate"] = int(command[10])
        return subprocess.CompletedProcess(command, 0)

    def writes(self):
        return [c for c in self.commands if "set" in c]

    def test_list_does_not_request_sudo_and_excludes_non_can(self):
        self.links["vcan0"] = {"ifname": "vcan0", "linkinfo": {"info_kind": "vcan"}}
        self.assertEqual(can_enable.main(["--list"]), 0)
        self.assertIn("can1", self.output.getvalue())
        self.assertNotIn("vcan0", self.output.getvalue())
        self.assertEqual(len(self.commands), 1)

    def test_dry_run_does_not_request_sudo_or_write(self):
        self.assertEqual(can_enable.main(["--all", "--dry-run", "--can-only"]), 0)
        self.assertEqual(len(self.commands), 1)
        self.assertEqual(self.output.getvalue().count("预览："), 4)

    def test_all_then_repeat_is_idempotent(self):
        self.assertEqual(can_enable.main(["--all", "--can-only"]), 0)
        self.assertEqual(len(self.writes()), 4)
        self.commands.clear()
        self.assertEqual(can_enable.main(["--all", "--can-only"]), 0)
        self.assertEqual(self.writes(), [])
        self.assertFalse(any("sudo" in c[0] for c in self.commands))

    def test_invalid_selection_or_up_mismatch_prevents_all_writes(self):
        with self.assertRaises(can_enable.CanError):
            can_enable.main(["can0", "absent", "--can-only"])
        self.links["can1"] = link("can1", True, 500000, "ERROR-ACTIVE", 5)
        with self.assertRaises(can_enable.CanError):
            can_enable.main(["--all", "--can-only"])
        self.assertEqual(self.writes(), [])
        self.assertFalse(any("sudo" in c[0] for c in self.commands))

    def test_bus_fault_or_unknown_state_is_not_restarted(self):
        for state in ("BUS-OFF", "ERROR-PASSIVE", "ERROR-WARNING", "未知"):
            self.links["can0"] = link(state=state)
            with self.assertRaises(can_enable.CanError):
                can_enable.main(["--all", "--can-only"])
        self.assertEqual(self.writes(), [])

    def test_changed_identity_after_password_stops_before_write(self):
        self.disconnect = True
        with self.assertRaises(can_enable.CanError):
            can_enable.main(["--all", "--can-only"])
        self.assertEqual(self.writes(), [])

    def test_failed_up_is_not_retried_and_second_interface_untouched(self):
        self.fail_up = True
        with self.assertRaises(can_enable.CanError):
            can_enable.main(["--all", "--can-only"])
        self.assertEqual(len(self.writes()), 2)
        self.assertTrue(all(c[6] == "can0" for c in self.writes()))
        self.assertIn("STOPPED", str(self.links["can1"]))

    def test_verification_fault_stops_before_next_interface(self):
        self.fail_verification = True
        with self.assertRaises(can_enable.CanError):
            can_enable.main(["--all", "--can-only"])
        self.assertEqual(len(self.writes()), 2)

    def test_interactive_selection_deduplicates(self):
        with patch.object(can_enable.sys.stdin, "isatty", return_value=True), \
                patch("builtins.input", return_value="2 can1"):
            self.assertEqual(can_enable.main(["--can-only"]), 0)
        self.assertEqual(len(self.writes()), 2)
        self.assertTrue(all(c[6] == "can1" for c in self.writes()))

    def test_cancel_and_empty_discovery_do_not_write(self):
        with patch.object(can_enable.sys.stdin, "isatty", return_value=True), \
                patch("builtins.input", return_value="q"):
            self.assertEqual(can_enable.main(["--can-only"]), 0)
        self.links.clear()
        self.assertEqual(can_enable.main(["--all", "--can-only"]), 1)
        self.assertEqual(self.writes(), [])

    def test_default_all_opens_network_then_invokes_motor_helper_once(self):
        self.assertEqual(can_enable.main(["--all"]), 0)
        self.assertEqual(len(self.writes()), 4)
        self.assertEqual(len(self.motor_calls), 1)
        self.assertEqual(self.motor_calls[0][-2:], ["can0", "can1"])

    def test_single_selection_opens_peer_network_but_selects_one_motor_arm(self):
        self.assertEqual(can_enable.main(["can1"]), 0)
        self.assertEqual(len(self.writes()), 4)
        self.assertEqual(len(self.motor_calls), 1)
        self.assertEqual(self.motor_calls[0][-1], "can1")
        self.assertNotIn("can0", self.motor_calls[0])

    def test_already_up_network_still_reaches_motor_stage(self):
        self.links = {name: link(name, True, 1000000, "ERROR-ACTIVE", i + 4)
                      for i, name in enumerate(self.links)}
        self.assertEqual(can_enable.main(["--all"]), 0)
        self.assertEqual(self.writes(), [])
        self.assertEqual(len(self.motor_calls), 1)

    def test_fault_does_not_prevent_motor_state_read_on_already_up_network(self):
        self.motor_preflight_error = True
        self.links = {name: link(name, True, 1000000, "ERROR-ACTIVE", i + 4)
                      for i, name in enumerate(self.links)}
        self.assertEqual(can_enable.main(["--all"]), 0)
        self.assertEqual(self.writes(), [])
        self.assertEqual(len(self.motor_calls), 1)
        self.assertFalse(any("--check-startup" in command for command in self.commands))

    def test_binding_error_still_blocks_even_when_network_already_up(self):
        self.binding_error = True
        self.links = {name: link(name, True, 1000000, "ERROR-ACTIVE", i + 4)
                      for i, name in enumerate(self.links)}
        with self.assertRaisesRegex(can_enable.CanError, "USB binding mismatch"):
            can_enable.main(["--all"])
        self.assertEqual(self.writes(), [])
        self.assertEqual(self.motor_calls, [])

    def test_network_failure_never_invokes_motor_enable(self):
        self.fail_up = True
        with self.assertRaises(can_enable.CanError):
            can_enable.main(["--all"])
        self.assertEqual(self.motor_calls, [])

    def test_owner_preflight_refusal_happens_before_network_mutation(self):
        self.motor_preflight_error = True
        with self.assertRaisesRegex(can_enable.CanError, "persistent owner/fault"):
            can_enable.main(["--all"])
        self.assertEqual(self.writes(), [])
        self.assertEqual(self.motor_calls, [])

    def test_combined_preview_never_invokes_hardware_helper(self):
        self.assertEqual(can_enable.main(["--all", "--dry-run"]), 0)
        self.assertEqual(self.writes(), [])
        self.assertEqual(self.motor_calls, [])
        self.assertTrue(any("--plan" in c for c in self.commands))
        self.assertFalse(any("--check" in c for c in self.commands))

    def test_default_resets_software_before_network_and_motor_startup(self):
        can_enable.main(["--all"])
        reset_index = next(i for i,c in enumerate(self.commands) if "--reset-state" in c)
        first_write = next(i for i,c in enumerate(self.commands) if "set" in c)
        self.assertLess(reset_index, first_write)
        self.assertEqual(sum("--reset-state" in c for c in self.commands), 1)

    def test_reset_only_does_not_open_network_or_enable_motors(self):
        can_enable.main(["--all", "--reset-only"])
        self.assertTrue(any("--reset-state" in c for c in self.commands))
        self.assertEqual(self.writes(), [])
        self.assertEqual(self.motor_calls, [])

    def test_keep_state_and_readonly_modes_do_not_reset(self):
        for arguments in (["--all", "--keep-state"], ["--all", "--can-only"],
                          ["--all", "--dry-run"], ["--list"]):
            self.commands.clear()
            can_enable.main(arguments)
            self.assertFalse(any("--reset-state" in c for c in self.commands))


if __name__ == "__main__":
    unittest.main()
