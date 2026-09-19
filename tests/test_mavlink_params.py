"""Tests for mavlink_params: expected-set construction, validation, and drift.

Network-free: PARAM_SET/READ paths are exercised indirectly through fake
`ParamClient.read` and `.write` overrides. What we care about is the
plan-defined behavior: expected values by SERIAL X/Y, synonym fallback for
`GPS1_TYPE` -> `GPS_TYPE`, and drift diffs producing per-parameter rows.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# Ensure app/ is importable
repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root / 'app'))

from tests import _stubs
_stubs.install()

import mavlink_params as mp  # noqa: E402


class ValidateSelectionTests(unittest.TestCase):
    def test_valid_selection(self):
        ok, err = mp.validate_selection(2, 3)
        self.assertTrue(ok, err)

    def test_same_index_rejected(self):
        ok, err = mp.validate_selection(3, 3)
        self.assertFalse(ok)
        self.assertIn('different', err)

    def test_out_of_range_rejected(self):
        ok, _ = mp.validate_selection(0, 3)
        self.assertFalse(ok)
        ok, _ = mp.validate_selection(2, 99)
        self.assertFalse(ok)

    def test_non_int_rejected(self):
        ok, _ = mp.validate_selection('2', 3)
        self.assertFalse(ok)
        ok, _ = mp.validate_selection(True, 3)
        self.assertFalse(ok, 'True must not be accepted as SERIAL index')


class ExpectedParamsTests(unittest.TestCase):
    def test_default_expected_set(self):
        exp = mp.build_expected_params(2, 3, use_gps_yaw_fallback=False)
        # Serial roles per plan.
        self.assertEqual(exp['SERIAL2_PROTOCOL'], float(mp.SERIAL_PROTOCOL_WINDVANE))
        self.assertEqual(exp['SERIAL3_PROTOCOL'], float(mp.SERIAL_PROTOCOL_GPS))
        # Wind vane both direction + speed types.
        self.assertEqual(exp['WNDVN_TYPE'], float(mp.WNDVN_TYPE_NMEA))
        self.assertEqual(exp['WNDVN_SPEED_TYPE'], float(mp.WNDVN_TYPE_NMEA))
        # NMEA GPS driver.
        self.assertEqual(exp['GPS1_TYPE'], float(mp.GPS_TYPE_NMEA))
        # SRC2 must be a complete set: yaw + posxy + velxy.
        self.assertEqual(exp['EK3_SRC2_YAW'], float(mp.EK3_YAW_GPS))
        self.assertEqual(exp['EK3_SRC2_POSXY'], float(mp.EK3_POSXY_GPS))
        self.assertEqual(exp['EK3_SRC2_VELXY'], float(mp.EK3_VELXY_GPS))
        # SRC1_YAW must NOT be touched by default (compass).
        self.assertNotIn('EK3_SRC1_YAW', exp)

    def test_opt_in_gps_yaw_fallback_adds_src1_yaw(self):
        exp = mp.build_expected_params(2, 3, use_gps_yaw_fallback=True)
        self.assertEqual(
            exp['EK3_SRC1_YAW'],
            float(mp.EK3_YAW_GPS_WITH_COMPASS_FALLBACK),
        )

    def test_expected_uses_selected_serial_indexes(self):
        exp = mp.build_expected_params(5, 7)
        self.assertIn('SERIAL5_PROTOCOL', exp)
        self.assertIn('SERIAL7_PROTOCOL', exp)


class SynonymTests(unittest.TestCase):
    def test_gps1_type_prefers_modern_name(self):
        self.assertEqual(mp._param_synonyms('GPS1_TYPE'), ['GPS1_TYPE', 'GPS_TYPE'])

    def test_other_params_have_no_synonyms(self):
        self.assertEqual(mp._param_synonyms('SERIAL3_PROTOCOL'), ['SERIAL3_PROTOCOL'])


class _FakeClient(mp.ParamClient):
    """ParamClient with `read` and `write` replaced by an in-memory store."""

    def __init__(self, existing=None):
        # Skip HTTP setup.
        self.existing = dict(existing or {})
        self.writes = []

    def read(self, name, timeout_s=3.0):
        return self.existing.get(name)

    def write(self, name, value, timeout_s=5.0):
        self.writes.append((name, value))
        self.existing[name] = float(value)
        return True, 'ok'


class ApplyExpectedTests(unittest.TestCase):
    def test_apply_wrote_and_noop_and_skipped(self):
        # Autopilot already has SERIAL2_PROTOCOL=21 (noop) and GPS_TYPE
        # (legacy) but not GPS1_TYPE. WNDVN_* absent -> skipped.
        client = _FakeClient({
            'SERIAL2_PROTOCOL': 21.0,
            'SERIAL3_PROTOCOL': 0.0,
            'GPS_TYPE': 0.0,          # will be resolved as synonym for GPS1_TYPE
            'EK3_SRC2_YAW': 1.0,
            'EK3_SRC2_POSXY': 0.0,
            'EK3_SRC2_VELXY': 0.0,
        })
        exp = mp.build_expected_params(2, 3, False)
        result = client.apply_expected(exp)

        self.assertEqual(result['SERIAL2_PROTOCOL']['action'], 'noop')
        self.assertTrue(result['SERIAL2_PROTOCOL']['ok'])

        self.assertEqual(result['SERIAL3_PROTOCOL']['action'], 'wrote')
        self.assertTrue(result['SERIAL3_PROTOCOL']['ok'])

        # GPS1_TYPE resolves to legacy GPS_TYPE and gets written.
        self.assertEqual(result['GPS1_TYPE']['resolved_name'], 'GPS_TYPE')
        self.assertEqual(result['GPS1_TYPE']['action'], 'wrote')

        # WNDVN_TYPE / WNDVN_SPEED_TYPE missing -> skipped, not failed.
        self.assertEqual(result['WNDVN_TYPE']['action'], 'skipped')
        self.assertFalse(result['WNDVN_TYPE']['available'])
        self.assertFalse(result['WNDVN_TYPE']['ok'])

    def test_snapshot_only_contains_available_params(self):
        client = _FakeClient({
            'SERIAL2_PROTOCOL': 0.0,
            'SERIAL3_PROTOCOL': 0.0,
            # WNDVN_* and GPS types missing -> should NOT appear in the snapshot.
            'EK3_SRC2_YAW': 0.0,
            'EK3_SRC2_POSXY': 0.0,
            'EK3_SRC2_VELXY': 0.0,
        })
        exp = mp.build_expected_params(2, 3, False)
        result = client.apply_expected(exp)
        snap = mp.snapshot_from_apply_result(result)
        self.assertIn('SERIAL2_PROTOCOL', snap)
        self.assertIn('SERIAL3_PROTOCOL', snap)
        self.assertNotIn('WNDVN_TYPE', snap)
        self.assertNotIn('GPS1_TYPE', snap)


class DriftTests(unittest.TestCase):
    def test_matches_produce_no_drift(self):
        client = _FakeClient({
            'SERIAL2_PROTOCOL': 21.0,
            'EK3_SRC2_YAW': 2.0,
        })
        expected = {'SERIAL2_PROTOCOL': 21.0, 'EK3_SRC2_YAW': 2.0}
        current = client.read_expected(expected)
        drift = mp.diff_current_vs_expected(current, expected)
        self.assertEqual(drift, [])

    def test_user_change_shows_up_as_drift(self):
        client = _FakeClient({
            'SERIAL2_PROTOCOL': 5.0,   # user set it to GPS
            'EK3_SRC2_YAW': 2.0,
        })
        expected = {'SERIAL2_PROTOCOL': 21.0, 'EK3_SRC2_YAW': 2.0}
        current = client.read_expected(expected)
        drift = mp.diff_current_vs_expected(current, expected)
        self.assertEqual(len(drift), 1)
        self.assertEqual(drift[0]['param'], 'SERIAL2_PROTOCOL')
        self.assertEqual(drift[0]['expected'], 21.0)
        self.assertEqual(drift[0]['current'], 5.0)
        self.assertTrue(drift[0]['available'])

    def test_missing_param_is_reported_as_unavailable_drift(self):
        client = _FakeClient({})  # no params at all
        expected = {'WNDVN_TYPE': 4.0}
        current = client.read_expected(expected)
        drift = mp.diff_current_vs_expected(current, expected)
        self.assertEqual(len(drift), 1)
        self.assertFalse(drift[0]['available'])
        self.assertIsNone(drift[0]['current'])


class ParamValueParsingTests(unittest.TestCase):
    def test_extract_from_naked_message(self):
        msg = {
            'type': 'PARAM_VALUE',
            'param_id': list('WNDVN_TYPE') + ['\x00'] * (16 - len('WNDVN_TYPE')),
            'param_value': 4.0,
        }
        got = mp.ParamClient._extract_param_value({'message': msg})
        self.assertEqual(got, ('WNDVN_TYPE', 4.0))

    def test_extract_from_wrapped_message(self):
        msg = {
            'status': {'time': {'first_message': 0, 'last_message': 1}},
            'message': {
                'type': 'PARAM_VALUE',
                'param_id': 'GPS1_TYPE\x00\x00\x00\x00\x00\x00\x00',
                'param_value': 5,
            },
        }
        got = mp.ParamClient._extract_param_value(msg)
        self.assertEqual(got, ('GPS1_TYPE', 5.0))

    def test_extract_returns_none_for_junk(self):
        self.assertIsNone(mp.ParamClient._extract_param_value(None))
        self.assertIsNone(mp.ParamClient._extract_param_value({'message': {'type': 'HEARTBEAT'}}))
        self.assertIsNone(mp.ParamClient._extract_param_value({'message': {'type': 'PARAM_VALUE'}}))


if __name__ == '__main__':
    unittest.main()
