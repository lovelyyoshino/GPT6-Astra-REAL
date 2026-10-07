"""Pure live model/transport contract tests; no API, camera or ROS traffic."""
import base64
import json
from pathlib import Path
import tempfile
import unittest

from right_pick.fast_codex import build_codex_packet
from right_pick.fast_model import build_fast_payload
from right_pick.fast_policy import (FastPolicyError, compact_controller_state,
    controller_phase_spec, parse_response, response_schema, validate_phase_decision)
from right_pick.fast_ros import ROSRightArm, FastROSError, VENDOR_SHA256


def live_budget():
    return dict(max_translation_m=.003, max_rotation_rad=.01, max_speed_percent=1,
                max_waypoints=1, gripper_min_m=0., gripper_max_m=.055,
                max_effort_parameter_nm=.2, required_effort_parameter_nm=.2,
                allow_waypoint_chunks=False)


def state(phase):
    return dict(phase=phase, robot_state={"sampled_at": 100.},
                gripper_state={"opening_m": .035}, action_budget=live_budget())


class LiveCapabilitiesTests(unittest.TestCase):
    def test_approach_live_contract_and_schema_have_no_chunk_or_waypoint_branch(self):
        for phase in ("APPROACH_PEN", "APPROACH_HOLDER"):
            packet = state(phase)
            contract = controller_phase_spec(packet)
            schema = response_schema(controller_state=packet)
            self.assertFalse(contract["chunk_allowed"])
            self.assertEqual(contract["chunk_max_waypoints"], 0)
            self.assertNotIn("move_eef_chunk", contract["allowed_actions"])
            self.assertNotIn("move_eef_chunk", schema["properties"]["action"]["enum"])
            self.assertEqual(schema["properties"]["phase"]["enum"], [phase])
            variants = schema["properties"]["arguments"]["anyOf"]
            self.assertFalse(any("waypoints" in variant["properties"] for variant in variants))
            move = next(v["properties"] for v in variants if "pose_m_rad" in v["properties"])
            self.assertEqual(move["speed_percent"]["maximum"], 1)
            decision = dict(phase=phase, action="move_eef_chunk", confidence=.9,
                arguments={"waypoints": [[0, 0, .2, 0, 0, 0]], "speed_percent": 1, "next_phase": None})
            with self.assertRaises(FastPolicyError):
                validate_phase_decision(decision, phase, controller_state=packet)

    def test_gripper_schema_fixed_point_two_matches_semantic_validation(self):
        for phase, opening in (("GRASP", .01), ("RELEASE", .04), ("INIT", .04)):
            packet = state(phase)
            schema = response_schema(controller_state=packet)
            variant = next(v["properties"] for v in schema["properties"]["arguments"]["anyOf"] if "opening_m" in v["properties"])
            self.assertEqual(variant["effort_parameter_nm"], {"type": "number", "enum": [.2]})
            self.assertEqual(controller_phase_spec(packet)["gripper_effort_parameter_nm"], .2)
            for effort in (.1, .3, .2):
                decision = dict(phase=phase, action="gripper", confidence=.9,
                    arguments={"opening_m": opening, "effort_parameter_nm": effort})
                if effort == .2:
                    validate_phase_decision(decision, phase, controller_state=packet)
                else:
                    with self.assertRaises(FastPolicyError):
                        validate_phase_decision(decision, phase, controller_state=packet)
            if phase in ("GRASP", "RELEASE"):
                self.assertNotIn("advance", schema["properties"]["action"]["enum"])

    def test_absent_transport_capabilities_preserve_replay_contract(self):
        packet = state("APPROACH_PEN")
        del packet["action_budget"]["allow_waypoint_chunks"]
        del packet["action_budget"]["required_effort_parameter_nm"]
        self.assertIn("move_eef_chunk", controller_phase_spec(packet)["allowed_actions"])
        self.assertEqual(response_schema(controller_state=packet), response_schema())
        packet["phase"] = "GRASP"
        decision = dict(phase="GRASP", action="gripper", confidence=.9,
                        arguments={"opening_m": .01, "effort_parameter_nm": .1})
        validate_phase_decision(decision, "GRASP", controller_state=packet)

    def test_bad_capabilities_are_not_coerced(self):
        for changes in ({"allow_waypoint_chunks": "false"}, {"max_waypoints": 2},
                        {"required_effort_parameter_nm": 0}, {"required_effort_parameter_nm": True},
                        {"required_effort_parameter_nm": .3}):
            packet = state("INIT")
            packet["action_budget"].update(changes)
            with self.assertRaises(FastPolicyError):
                compact_controller_state(packet)

    def test_responses_and_codex_packets_advertise_the_same_supported_route(self):
        png = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII=")
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "rgb.png"
            image.write_bytes(png)
            observation = {"cameras": {name: {"rgb_path": str(image)} for name in ("front", "right_hand", "left_hand")}}
            for phase in ("APPROACH_PEN", "GRASP"):
                packet = state(phase)
                payload = build_fast_payload({"model_id": "gpt-6-astra", "protocol": "responses"}, packet, observation)
                api_text = json.loads(payload["input"][0]["content"][0]["text"])
                cli_text = build_codex_packet(packet, observation)
                self.assertEqual(api_text["phase_contract"], cli_text["phase_contract"])
                self.assertEqual(api_text["controller_state"], cli_text["controller_state"])
                self.assertEqual(payload["text"]["format"]["schema"], response_schema(controller_state=packet))
                self.assertNotIn("move_eef_chunk", api_text["phase_contract"]["allowed_actions"])
                for text in (api_text, cli_text):
                    self.assertIn("commanded robot endpoint", text["instruction"])
                    self.assertIn("bounded, visually supported exploratory move", text["instruction"])
                    self.assertIn("does not change the robot or camera pose", text["instruction"])

    def test_exact_gripper_effort_corresponds_to_pinned_ros_encoder(self):
        raw = dict(source="sdk_receive_raw_frames", can_interface="can1", driver_sha256=VENDOR_SHA256,
            sdk_version="0.6.2", stamps=[99.99]*14, stamp=99.995, sequence=100, source_sequence=100,
            raw_q=[0]*6, q=[0.]*6, pose=[.25, 0, .25, 0, 0, 0], opening_m=.035,
            gripper_torque_sdk_units=50, ctrl_mode=1, arm_status=0, mode=0, teach_status=0,
            motion_status=0, fault=0, driver_codes=[64]*6, jaw_code=64, enabled=[True]*6,
            active_command=False, driver_accepts_commands=True, failure=None)
        robot = ROSRightArm({"ros": {"command_log": "/offline/no-driver.log"}}, clock=lambda: 100.)
        measured = {"raw_telemetry": raw, "provenance": {"command_sequence": 0, "speed_percent": 1}}
        required = state("GRASP")["action_budget"]["required_effort_parameter_nm"]
        decision = dict(phase="GRASP", action="gripper", confidence=.9,
            arguments={"opening_m": .01, "effort_parameter_nm": required})
        envelope = robot.prepare_command(parse_response(decision), measured)
        self.assertEqual(envelope["request"]["gripper_effort"], required)
        self.assertEqual(envelope["expected_frames"][0]["data_hex"], "0000271000c80100")
        decision["arguments"]["effort_parameter_nm"] = .1
        with self.assertRaises(FastROSError):
            robot.prepare_command(parse_response(decision), measured)
        self.assertIsNone(robot._transport)


if __name__ == "__main__":
    unittest.main()
