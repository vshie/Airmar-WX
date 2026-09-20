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
        # Pin a target so apply_expected()'s up-front discovery is a no-op.
        self.target_system = 2
        self.target_component = 1
        self._target_pinned = True

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


class AutopilotDiscoveryTests(unittest.TestCase):
    """The autopilot is NOT reliably at (system 1, component 1).

    Captured live from the BlueBoat at 192.168.1.69 on 2026-09-20:
      sys=1  comp=191  MAV_AUTOPILOT_INVALID / MAV_TYPE_ONBOARD_CONTROLLER
      sys=2  comp=1    MAV_AUTOPILOT_ARDUPILOTMEGA / MAV_TYPE_SURFACE_BOAT
      sys=255 comp=*   this extension's own NAMED_VALUE_FLOAT publishers

    Hardcoding (1, 1) sent every PARAM_SET to a system no autopilot was
    listening on and left the PARAM_VALUE mailbox for (1, 1) permanently
    empty — which is exactly why apply appeared to "succeed" while
    changing nothing, and why reads took forever before timing out.
    """

    # Trimmed copy of the real /mavlink/vehicles response.
    REAL_TOPOLOGY = {
        "255": {"id": 255, "components": {
            "73": {"id": 73, "messages": {"NAMED_VALUE_FLOAT": {"message": {
                "type": "NAMED_VALUE_FLOAT", "value": 0.6}}}},
            "240": {"id": 240, "messages": {"PARAM_SET": {"message": {
                "type": "PARAM_SET"}}}},
        }},
        "2": {"id": 2, "components": {
            "1": {"id": 1, "messages": {"HEARTBEAT": {"message": {
                "type": "HEARTBEAT",
                "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                "mavtype": {"type": "MAV_TYPE_SURFACE_BOAT"}}}}},
            "194": {"id": 194, "messages": {"STATUSTEXT": {"message": {
                "type": "STATUSTEXT"}}}},
        }},
        "1": {"id": 1, "components": {
            "191": {"id": 191, "messages": {"HEARTBEAT": {"message": {
                "type": "HEARTBEAT",
                "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
                "mavtype": {"type": "MAV_TYPE_ONBOARD_CONTROLLER"}}}}},
        }},
    }

    def test_picks_real_autopilot_not_system_one(self):
        got = mp.pick_autopilot(self.REAL_TOPOLOGY,
                                prefer_mavtypes=mp.BOAT_MAVTYPES)
        self.assertEqual(got, (2, 1),
                         "must find the ArduPilot FC at sys 2, not sys 1")

    def test_ignores_companion_and_gcs_nodes(self):
        """BlueOS's onboard controller (sys 1) and our own NVF publisher
        (sys 255) must never be mistaken for the autopilot."""
        only_companions = {
            "1": self.REAL_TOPOLOGY["1"],
            "255": self.REAL_TOPOLOGY["255"],
        }
        self.assertIsNone(mp.pick_autopilot(only_companions))

    def test_prefers_boat_over_other_autopilots(self):
        topology = dict(self.REAL_TOPOLOGY)
        topology["3"] = {"id": 3, "components": {"1": {"id": 1, "messages": {
            "HEARTBEAT": {"message": {
                "type": "HEARTBEAT",
                "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                "mavtype": {"type": "MAV_TYPE_SUBMARINE"}}}}}}}
        got = mp.pick_autopilot(topology, prefer_mavtypes=mp.BOAT_MAVTYPES)
        self.assertEqual(got, (2, 1), "surface boat must win over submarine")

    def test_falls_back_to_any_autopilot_when_no_preference_matches(self):
        sub_only = {"3": {"id": 3, "components": {"1": {"id": 1, "messages": {
            "HEARTBEAT": {"message": {
                "type": "HEARTBEAT",
                "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                "mavtype": {"type": "MAV_TYPE_SUBMARINE"}}}}}}}}
        self.assertEqual(
            mp.pick_autopilot(sub_only, prefer_mavtypes=mp.BOAT_MAVTYPES),
            (3, 1),
        )

    def test_handles_bare_string_enums(self):
        """Older mavlink2rest emits enum fields as bare strings rather
        than {"type": "..."} wrappers."""
        topology = {"4": {"id": 4, "components": {"1": {"id": 1, "messages": {
            "HEARTBEAT": {"message": {
                "type": "HEARTBEAT",
                "autopilot": "MAV_AUTOPILOT_ARDUPILOTMEGA",
                "mavtype": "MAV_TYPE_GROUND_ROVER"}}}}}}}
        self.assertEqual(mp.pick_autopilot(topology), (4, 1))

    def test_junk_input_returns_none(self):
        self.assertIsNone(mp.pick_autopilot(None))
        self.assertIsNone(mp.pick_autopilot({}))
        self.assertIsNone(mp.pick_autopilot({"notanint": {"components": {}}}))

    def test_pinned_target_skips_discovery(self):
        """Passing explicit ids must pin them (tests rely on this)."""
        client = mp.ParamClient(base_url='http://fake', target_system=7,
                                target_component=3)
        self.assertTrue(client.ensure_target())
        self.assertEqual((client.target_system, client.target_component), (7, 3))

    def test_apply_reports_failed_when_no_autopilot_visible(self):
        """If discovery finds nothing, every row must be 'failed' with a
        clear reason — never a silent success that misleads the user."""
        class _NoAutopilotClient(mp.ParamClient):
            def __init__(self):
                self.base_url = 'http://fake/mavlink2rest'
                self.target_system = None
                self.target_component = None
                self._target_pinned = False
            def discover_target(self):
                return None

        client = _NoAutopilotClient()
        expected = mp.build_expected_params(6, 7, False)
        result = client.apply_expected(expected)
        self.assertEqual(set(result), set(expected))
        for name, row in result.items():
            self.assertEqual(row['action'], 'failed', name)
            self.assertFalse(row['ok'], name)
            self.assertIn('autopilot', row['reason'])
        # Nothing should be snapshotted from a fully failed apply.
        self.assertEqual(mp.snapshot_from_apply_result(result), {})


class _FakeResponse:
    def __init__(self, status_code=200, text=''):
        self.status_code = status_code
        self.text = text


class _FakeSession:
    """Records POSTs / GETs and returns canned responses per URL suffix.

    Suffix-match keys keep the tests readable — the real base URL
    (`http://host.docker.internal/mavlink2rest`) can be anything as long
    as the endpoint paths are correct.
    """

    def __init__(self, get_responses=None, post_responses=None):
        # dict: url suffix -> _FakeResponse (or callable returning one)
        self.get_responses = get_responses or {}
        self.post_responses = post_responses or {}
        self.gets = []
        self.posts = []

    def _match(self, table, url):
        for suffix, response in table.items():
            if url.endswith(suffix):
                return response(url) if callable(response) else response
        return _FakeResponse(404, '')

    def get(self, url, timeout=None):
        self.gets.append(url)
        return self._match(self.get_responses, url)

    def post(self, url, json=None, timeout=None):
        self.posts.append((url, json))
        return self._match(self.post_responses, url)


class Mavlink2RestTransportTests(unittest.TestCase):
    """The bug that shipped in 1.1.3: mavlink2rest returns HTTP 200 with
    body `"Failed to parse message, not a valid MAVLinkMessage."` when
    the payload schema is wrong. The old `_post` didn't check the body,
    so every rejected PARAM_SET looked like a success and the UI
    reported "wrote (unverified)" for writes that never went out."""

    def _make_client(self, session):
        # Pin the target so these tests exercise the transport only;
        # autopilot discovery has its own test class.
        client = mp.ParamClient(base_url='http://fake/mavlink2rest',
                                target_system=1, target_component=1)
        client._session = session
        return client

    def test_failed_body_is_treated_as_post_failure(self):
        session = _FakeSession(post_responses={
            '/mavlink': _FakeResponse(
                200,
                'Failed to parse message, not a valid MAVLinkMessage.',
            ),
        })
        client = self._make_client(session)
        posted, reason, verified = client.write('WNDVN_TYPE', 4.0, timeout_s=0.1)
        self.assertFalse(posted, "malformed PARAM_SET must NOT be reported ok")
        self.assertIn('rejected', reason.lower() + reason)
        self.assertFalse(verified)

    def test_http_500_is_treated_as_post_failure(self):
        session = _FakeSession(post_responses={
            '/mavlink': _FakeResponse(500, 'internal server error'),
        })
        client = self._make_client(session)
        posted, _, verified = client.write('WNDVN_TYPE', 4.0, timeout_s=0.1)
        self.assertFalse(posted)
        self.assertFalse(verified)

    def test_success_body_with_matching_echo_is_verified(self):
        """Full happy path: POST is accepted, and the PARAM_VALUE mailbox
        returns a body with matching name + value + a fresh stamp."""
        name = 'SERIAL7_PROTOCOL'
        # The mailbox response uses mavlink2rest's wrapping shape.
        mailbox_before = _FakeResponse(200, '')  # empty on first GET
        mailbox_after = _FakeResponse(200, __import__('json').dumps({
            'header': {'system_id': 1, 'component_id': 1, 'sequence': 0},
            'message': {
                'type': 'PARAM_VALUE',
                'param_id': list(name) + ['\x00'] * (16 - len(name)),
                'param_value': 21.0,
            },
            'status': {'time': {'last_update': 'after-write'}},
        }))
        # First call returns 'before', subsequent calls return 'after'.
        state = {'calls': 0}
        def mailbox(url):
            state['calls'] += 1
            return mailbox_before if state['calls'] == 1 else mailbox_after

        session = _FakeSession(
            get_responses={f'/messages/PARAM_VALUE': mailbox},
            post_responses={'/mavlink': _FakeResponse(200, 'ok')},
        )
        client = self._make_client(session)
        # Skip the /helper/mavlink template lookup by pre-caching an empty
        # template so build_envelope doesn't try to GET /helper.
        client._template_cache['PARAM_SET'] = {'message': {}}
        posted, reason, verified = client.write(name, 21.0, timeout_s=1.0)
        self.assertTrue(posted, reason)
        self.assertTrue(verified, reason)


class ParamIdDecodingTests(unittest.TestCase):
    """`_chars_to_str` is the extractor for PARAM_VALUE.param_id and
    it has to survive every shape mavlink2rest / mavlink-server has
    ever serialized a `char[16]` field as. This is the single place
    where a format mismatch turns every apply into a silent no-op."""

    def test_list_of_single_char_strings(self):
        """Python mavlink2rest and BlueOS's current mavlink-server both
        emit param_id as a list of single-char strings."""
        pid = list('WNDVN_TYPE') + ['\x00'] * (16 - len('WNDVN_TYPE'))
        self.assertEqual(mp._chars_to_str(pid), 'WNDVN_TYPE')

    def test_padded_string(self):
        """Some builds serialize the char array as a padded string."""
        pid = 'GPS1_TYPE' + '\x00' * (16 - len('GPS1_TYPE'))
        self.assertEqual(mp._chars_to_str(pid), 'GPS1_TYPE')

    def test_list_of_byte_ints(self):
        """Defensive: rust-mavlink can serialize [u8;16] as a JSON list
        of ASCII byte integers. Handle it so a future serializer flip
        doesn't silently break apply again."""
        name = 'SERIAL7_PROTOCOL'
        pid = [ord(c) for c in name] + [0] * (16 - len(name))
        self.assertEqual(mp._chars_to_str(pid), 'SERIAL7_PROTOCOL')

    def test_junk_inputs_return_empty(self):
        self.assertEqual(mp._chars_to_str(None), '')
        self.assertEqual(mp._chars_to_str([]), '')
        self.assertEqual(mp._chars_to_str([0, 0, 0]), '')

    def test_non_printable_bytes_are_dropped(self):
        # 0xFF is not printable ASCII; must not turn into a param name.
        self.assertEqual(mp._chars_to_str([0xFF, 0x00, 0x01]), '')

    def test_str_to_chars_round_trip(self):
        chars = mp._str_to_chars('WNDVN_TYPE')
        self.assertEqual(len(chars), mp.PARAM_ID_LEN)
        self.assertEqual(mp._chars_to_str(chars), 'WNDVN_TYPE')


class ParamValueDecodingTests(unittest.TestCase):
    """`ParamClient._decode_param_value` takes the PARAM_VALUE message
    body (already unwrapped from the mavlink2rest envelope) and returns
    (name, float) so higher-level code doesn't have to care about
    serializer quirks in `param_value`."""

    def test_plain_float(self):
        body = {
            'param_id': list('WNDVN_TYPE') + ['\x00'] * (16 - 10),
            'param_value': 4.0,
        }
        name, value = mp.ParamClient._decode_param_value(body)
        self.assertEqual(name, 'WNDVN_TYPE')
        self.assertEqual(value, 4.0)

    def test_int_value(self):
        body = {
            'param_id': list('GPS1_TYPE') + ['\x00'] * (16 - 9),
            'param_value': 5,
        }
        name, value = mp.ParamClient._decode_param_value(body)
        self.assertEqual(name, 'GPS1_TYPE')
        self.assertEqual(value, 5.0)

    def test_wrapped_value(self):
        """Some builds emit `param_value` as `{"type": "...", "value": ...}`."""
        body = {
            'param_id': list('SERIAL7_PROTOCOL'),
            'param_value': {'type': 'MAV_PARAM_TYPE_REAL32', 'value': 21.0},
        }
        name, value = mp.ParamClient._decode_param_value(body)
        self.assertEqual(name, 'SERIAL7_PROTOCOL')
        self.assertEqual(value, 21.0)

    def test_missing_value_returns_none(self):
        body = {'param_id': list('X') + ['\x00'] * 15}
        _, value = mp.ParamClient._decode_param_value(body)
        self.assertIsNone(value)


if __name__ == '__main__':
    unittest.main()
