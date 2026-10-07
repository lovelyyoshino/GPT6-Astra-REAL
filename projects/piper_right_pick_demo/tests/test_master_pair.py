"""Compare recorded inputs without authorizing follow or loading hardware."""
import importlib.util
import sys
from pathlib import Path
import unittest

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('master_pair_test', scripts / 'check_master_pair.py')
pair_check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pair_check)
sys.path.remove(str(scripts))


def evidence():
    master = {'command_feedback': {}}
    right = {'feedback': {}}
    for pair in ('12', '34', '56'):
        common = {'age_s_at_finish': .01, 'received_monotonic_s': 100., 'origin': 'nonlocal'}
        m = dict(common, fields={'joint_' + n: 1000 * int(n) for n in pair})
        r = dict(common, fields={'joint_' + n: 1000 * int(n) - 100 for n in pair})
        master['command_feedback']['PiperMsgJointCtrl_' + pair] = {'latest_by_origin': {'nonlocal': m}}
        right['feedback']['PiperMsgJointFeedBack_' + pair] = r
    return master, right


class MasterPairTests(unittest.TestCase):
    def test_degrees_from_actual_fields_never_grant_follow(self):
        result = pair_check.compare(*evidence())
        self.assertTrue(result['comparison_available'])
        self.assertAlmostEqual(result['max_absolute_difference_deg'], .1)
        self.assertEqual(result['master_target_deg'], [1., 2., 3., 4., 5., 6.])
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['follow_ready'])

    def test_missing_or_local_master_does_not_become_zero_target(self):
        master, right = evidence()
        master['command_feedback']['PiperMsgJointCtrl_12']['latest_by_origin'] = {'local': {}}
        result = pair_check.compare(master, right)
        self.assertFalse(result['comparison_available'])
        self.assertNotIn('master_target_deg', result)

    def test_stale_or_unsynchronized_frames_rejected(self):
        master, right = evidence()
        right['feedback']['PiperMsgJointFeedBack_12']['age_s_at_finish'] = 1
        self.assertFalse(pair_check.compare(master, right)['comparison_available'])
        master, right = evidence()
        right['feedback']['PiperMsgJointFeedBack_12']['received_monotonic_s'] = 101.
        self.assertFalse(pair_check.compare(master, right)['comparison_available'])

    def test_local_loopback_is_not_right_hardware_feedback(self):
        master, right = evidence()
        right['feedback']['PiperMsgJointFeedBack_12']['origin'] = 'local'
        self.assertFalse(pair_check.compare(master, right)['comparison_available'])


if __name__ == '__main__':
    unittest.main()
