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
    """ParamClient with `read` and `write` replaced by an in-memory store.

    Simulates the important real-world edge case: the mavlink2rest PARAM_VALUE
    echo path can be flaky even when writes go through. Callers can pass
    `readable=False` to model "writes work, reads all time out" — that used
    to make `apply_expected` report every param as "not found on autopilot",
    which is the bug we're guarding against here.
    """

    def __init__(self, existing=None, readable=True, write_fails=None):
        # Skip HTTP setup.
        self.existing = dict(existing or {})
        self.readable = readable
        self.write_fails = set(write_fails or ())
        self.writes = []

    def read(self, name, timeout_s=3.0):
        if not self.readable:
            return None
        return self.existing.get(name)

    def write(self, name, value, timeout_s=5.0):
        self.writes.append((name, value))
        if name in self.write_fails:
            return False, 'simulated POST failure', False
        self.existing[name] = float(value)
        # Model "posted and verified" by default; the transport-unreliable
        # variant is `_FakeClient(readable=False)` which also implies verify
        # can't complete (no echo mailbox to poll).
        return True, 'ok', bool(self.readable)


class ApplyExpectedTests(unittest.TestCase):
    def test_apply_wrote_and_noop(self):
        # Autopilot already has SERIAL2_PROTOCOL=21 (noop). GPS_TYPE
        # (legacy) resolves as a synonym for GPS1_TYPE.
        client = _FakeClient({
            'SERIAL2_PROTOCOL': 21.0,
            'SERIAL3_PROTOCOL': 0.0,
            'GPS_TYPE': 0.0,          # legacy alias present, GPS1_TYPE not
            'EK3_SRC2_YAW': 1.0,
            'EK3_SRC2_POSXY': 0.0,
            'EK3_SRC2_VELXY': 0.0,
            'WNDVN_TYPE': 0.0,
            'WNDVN_SPEED_TYPE': 0.0,
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

        self.assertEqual(result['WNDVN_TYPE']['action'], 'wrote')
        self.assertTrue(result['WNDVN_TYPE']['ok'])

    def test_unreadable_transport_still_posts_writes(self):
        """Regression: when the PARAM_VALUE echo transport is broken,
        apply_expected must NOT report every param as "not found" — it
        must still POST the PARAM_SETs and mark them as wrote_unverified.
        """
        client = _FakeClient(existing={
            # These exist on the FC but reads all fail (readable=False).
            'SERIAL7_PROTOCOL': 0.0, 'SERIAL8_PROTOCOL': 0.0,
            'WNDVN_TYPE': 0.0, 'WNDVN_SPEED_TYPE': 0.0,
            'GPS1_TYPE': 0.0, 'EK3_SRC2_YAW': 0.0,
            'EK3_SRC2_POSXY': 0.0, 'EK3_SRC2_VELXY': 0.0,
        }, readable=False)
        exp = mp.build_expected_params(7, 8, False)
        result = client.apply_expected(exp)

        # Every row must be a successful post, unverified because reads
        # are broken — but crucially not 'skipped' with "not found".
        for name, row in result.items():
            self.assertTrue(row['ok'], f"{name} should be ok: {row}")
            self.assertEqual(
                row['action'], 'wrote_unverified',
                f"{name} should be wrote_unverified: {row}",
            )
        # And every write actually happened on the fake FC.
        written = {name for name, _ in client.writes}
        self.assertIn('SERIAL7_PROTOCOL', written)
        self.assertIn('SERIAL8_PROTOCOL', written)
        self.assertIn('WNDVN_TYPE', written)

    def test_post_failure_marks_failed(self):
        client = _FakeClient(
            existing={'SERIAL2_PROTOCOL': 0.0, 'SERIAL3_PROTOCOL': 0.0,
                      'WNDVN_TYPE': 0.0, 'WNDVN_SPEED_TYPE': 0.0,
                      'GPS1_TYPE': 0.0, 'EK3_SRC2_YAW': 0.0,
                      'EK3_SRC2_POSXY': 0.0, 'EK3_SRC2_VELXY': 0.0},
            write_fails={'WNDVN_TYPE'},
        )
        exp = mp.build_expected_params(2, 3, False)
        result = client.apply_expected(exp)
        self.assertEqual(result['WNDVN_TYPE']['action'], 'failed')
        self.assertFalse(result['WNDVN_TYPE']['ok'])
        # Other params still went through.
        self.assertEqual(result['SERIAL2_PROTOCOL']['action'], 'wrote')

    def test_snapshot_includes_verified_and_unverified_writes(self):
        # Unverified writes MUST be snapshotted so the Check step later
        # can surface truth — that's the whole point of the drift check.
        client = _FakeClient(existing={
            'SERIAL2_PROTOCOL': 0.0, 'SERIAL3_PROTOCOL': 0.0,
            'WNDVN_TYPE': 0.0, 'WNDVN_SPEED_TYPE': 0.0,
            'GPS1_TYPE': 0.0, 'EK3_SRC2_YAW': 0.0,
            'EK3_SRC2_POSXY': 0.0, 'EK3_SRC2_VELXY': 0.0,
        }, readable=False)
        exp = mp.build_expected_params(2, 3, False)
        result = client.apply_expected(exp)
        snap = mp.snapshot_from_apply_result(result)
        # Every expected param made it into the snapshot.
        for k in exp:
            self.assertIn(k, snap, f"{k} missing from snapshot: {snap}")

    def test_snapshot_excludes_failed_posts(self):
        client = _FakeClient(
            existing={'SERIAL2_PROTOCOL': 0.0, 'SERIAL3_PROTOCOL': 0.0,
                      'WNDVN_TYPE': 0.0, 'WNDVN_SPEED_TYPE': 0.0,
                      'GPS1_TYPE': 0.0, 'EK3_SRC2_YAW': 0.0,
                      'EK3_SRC2_POSXY': 0.0, 'EK3_SRC2_VELXY': 0.0},
            write_fails={'WNDVN_TYPE'},
        )
        exp = mp.build_expected_params(2, 3, False)
        result = client.apply_expected(exp)
        snap = mp.snapshot_from_apply_result(result)
        self.assertNotIn('WNDVN_TYPE', snap)
        self.assertIn('SERIAL2_PROTOCOL', snap)


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
    def test_extract_from_list_of_chars(self):
        """Python mavlink2rest emits param_id as a list of 1-char strings."""
        msg = {
            'type': 'PARAM_VALUE',
            'param_id': list('WNDVN_TYPE') + ['\x00'] * (16 - len('WNDVN_TYPE')),
            'param_value': 4.0,
        }
        got = mp.ParamClient._extract_param_value({'message': msg})
        self.assertEqual(got, ('WNDVN_TYPE', 4.0))

    def test_extract_from_list_of_byte_ints(self):
        """rust-mavlink (BlueOS ≥1.2) emits param_id as a list of i8 byte
        values. This is the shape that broke the previous release: the old
        extractor filtered them out with `isinstance(c, str)` and returned
        None, which cascaded into "parameter not found on autopilot".
        """
        name = 'EK3_SRC2_POSXY'
        bytes_list = [ord(c) for c in name] + [0] * (16 - len(name))
        msg = {
            'header': {'system_id': 1, 'component_id': 1, 'sequence': 0},
            'message': {
                'type': 'PARAM_VALUE',
                'param_id': bytes_list,
                'param_value': 3.0,
                'param_type': {'type': 'MAV_PARAM_TYPE_REAL32'},
            },
        }
        got = mp.ParamClient._extract_param_value(msg)
        self.assertEqual(got, ('EK3_SRC2_POSXY', 3.0))

    def test_extract_from_list_of_signed_bytes(self):
        """i8 values arriving as-is from JSON serializers can include the
        signed range for the padding; verify the low byte is used."""
        name = 'GPS1_TYPE'
        # Fill the rest with a negative "0" (i.e. straight 0). Also splice a
        # signed negative that maps to a printable ASCII byte to make sure
        # we don't overreact to it.
        bytes_list = [ord(c) for c in name] + [0] * (16 - len(name))
        got = mp.ParamClient._extract_param_value({'message': {
            'type': 'PARAM_VALUE',
            'param_id': bytes_list,
            'param_value': 5.0,
        }})
        self.assertEqual(got, ('GPS1_TYPE', 5.0))

    def test_extract_from_string_param_id(self):
        """Some builds serialize the char array as a padded string."""
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

    def test_extract_from_wrapped_dict_param_id(self):
        """Some serializers wrap fixed-size arrays as {"data": [...]}"""
        name = 'SERIAL7_PROTOCOL'
        bytes_list = [ord(c) for c in name] + [0] * (16 - len(name))
        msg = {
            'message': {
                'type': 'PARAM_VALUE',
                'param_id': {'data': bytes_list},
                'param_value': 21.0,
            },
        }
        got = mp.ParamClient._extract_param_value(msg)
        self.assertEqual(got, ('SERIAL7_PROTOCOL', 21.0))

    def test_extract_from_wrapped_param_value(self):
        """param_value may arrive wrapped as {"type": "...", "value": 21.0}"""
        name = 'SERIAL7_PROTOCOL'
        bytes_list = [ord(c) for c in name] + [0] * (16 - len(name))
        got = mp.ParamClient._extract_param_value({'message': {
            'type': 'PARAM_VALUE',
            'param_id': bytes_list,
            'param_value': {'type': 'MAV_PARAM_TYPE_REAL32', 'value': 21.0},
        }})
        self.assertEqual(got, ('SERIAL7_PROTOCOL', 21.0))

    def test_extract_returns_none_for_junk(self):
        self.assertIsNone(mp.ParamClient._extract_param_value(None))
        self.assertIsNone(mp.ParamClient._extract_param_value({}))
        self.assertIsNone(mp.ParamClient._extract_param_value(
            {'message': {'type': 'HEARTBEAT'}}
        ))
        # PARAM_VALUE with no param_id/param_value should not be accepted.
        self.assertIsNone(mp.ParamClient._extract_param_value(
            {'message': {'type': 'PARAM_VALUE'}}
        ))
        # Non-printable bytes should not be interpreted as a param name.
        self.assertIsNone(mp.ParamClient._extract_param_value({'message': {
            'type': 'PARAM_VALUE',
            'param_id': [0xFF, 0x00, 0x01],
            'param_value': 1.0,
        }}))


if __name__ == '__main__':
    unittest.main()
