"""BlueOS serial-list planning and the ardupilot-manager client.

The live vehicle this was built against had
    B=/dev/ttyS0, D=udpin:0.0.0.0:27000 (stale), E=<usb device>,
    G=udpin:0.0.0.0:27001
so several cases below use that list.
"""

import json
import sys
import unittest
from pathlib import Path

from tests import _stubs
_stubs.install()

_APP = Path(__file__).resolve().parent.parent / 'app'
if str(_APP) not in sys.path:
    sys.path.insert(0, str(_APP))

import blueos_serials as bs  # noqa: E402

WIND = 'udpin:0.0.0.0:27001'
GPS = 'udpin:0.0.0.0:27002'
USB = '/dev/serial/by-path/platform-usb-0:1.3:1.0-port0'
LIVE = [
    {'port': 'D', 'endpoint': 'udpin:0.0.0.0:27000'},
    {'port': 'E', 'endpoint': USB},
    {'port': 'B', 'endpoint': '/dev/ttyS0'},
    {'port': 'G', 'endpoint': WIND},
]


def by_letter(entries):
    return {e['port']: e['endpoint'] for e in entries}


def kinds(changes):
    return {(c['letter'], c['kind']) for c in changes}


class MappingTests(unittest.TestCase):
    def test_letter_mapping_matches_blueos(self):
        # core/frontend/.../AutopilotSerialConfiguration.vue
        self.assertEqual(bs.SERIAL_TO_LETTER,
                         {1: 'C', 2: 'D', 3: 'B', 4: 'E', 5: 'F', 6: 'G', 7: 'H'})
        self.assertNotIn('A', bs.LETTER_TO_SERIAL)

    def test_endpoint_udp_port(self):
        self.assertEqual(bs.endpoint_udp_port('udpin:0.0.0.0:27001'), 27001)
        self.assertEqual(bs.endpoint_udp_port('udp:192.168.2.1:27002'), 27002)
        self.assertIsNone(bs.endpoint_udp_port('/dev/ttyS0'))
        self.assertIsNone(bs.endpoint_udp_port('tcpin:0.0.0.0:27001'))
        self.assertIsNone(bs.endpoint_udp_port(None))

    def test_validate_auto_serials_rejects_8_and_9(self):
        self.assertTrue(bs.validate_auto_serials(6, 7)[0])
        self.assertTrue(bs.validate_auto_serials(None, 7)[0])
        self.assertFalse(bs.validate_auto_serials(8, None)[0])
        self.assertFalse(bs.validate_auto_serials(6, 9)[0])

    def test_is_supported(self):
        self.assertTrue(bs.is_supported({'name': 'Navigator'}, LIVE)[0])
        self.assertTrue(bs.is_supported({'name': 'Navigator'}, [])[0])
        ok, reason = bs.is_supported({'name': 'Pixhawk1'}, [])
        self.assertFalse(ok)
        self.assertIn('Pixhawk1', reason)
        self.assertFalse(bs.is_supported({'name': 'SITL'}, [])[0])
        self.assertFalse(bs.is_supported(None, None)[0])
        self.assertFalse(bs.is_supported({'name': 'Navigator'}, None)[0])

    def test_dead_device_entries(self):
        dead = bs.dead_device_entries(LIVE, exists=lambda p: p == '/dev/ttyS0')
        self.assertEqual(dead, [{'port': 'E', 'endpoint': USB}])


class PlanTests(unittest.TestCase):
    def test_live_vehicle_adds_gps_on_serial7_only(self):
        new, changes = bs.plan_serial_changes(LIVE, 6, 7, WIND, GPS)
        self.assertEqual(kinds(changes), {('G', 'unchanged'), ('H', 'set')})
        self.assertEqual(new, LIVE + [{'port': 'H', 'endpoint': GPS}])

    def test_sets_empty_slots(self):
        new, changes = bs.plan_serial_changes([], 6, 7, WIND, GPS)
        self.assertEqual(new, [{'port': 'G', 'endpoint': WIND},
                               {'port': 'H', 'endpoint': GPS}])
        self.assertEqual(kinds(changes), {('G', 'set'), ('H', 'set')})

    def test_unchanged_when_already_configured(self):
        cur = [{'port': 'G', 'endpoint': WIND}, {'port': 'H', 'endpoint': GPS}]
        new, changes = bs.plan_serial_changes(cur, 6, 7, WIND, GPS)
        self.assertEqual(new, cur)
        self.assertTrue(all(c['kind'] == 'unchanged' for c in changes))

    def test_replaces_foreign_entry_on_target(self):
        cur = [{'port': 'H', 'endpoint': '/dev/ttyAMA4'}]
        new, changes = bs.plan_serial_changes(cur, None, 7, WIND, GPS)
        self.assertEqual(new, [{'port': 'H', 'endpoint': GPS}])
        self.assertEqual(changes[0]['kind'], 'replaced')
        self.assertEqual(changes[0]['before'], '/dev/ttyAMA4')

    def test_moves_endpoint_from_other_letter(self):
        # Wind feed is on SERIAL6 (G) but the user picks SERIAL5 (F).
        new, changes = bs.plan_serial_changes(LIVE, 5, None, WIND, GPS)
        self.assertNotIn('G', by_letter(new))
        self.assertEqual(by_letter(new)['F'], WIND)
        self.assertEqual(kinds(changes), {('G', 'moved-from'), ('F', 'set')})
        moved = [c for c in changes if c['kind'] == 'moved-from'][0]
        self.assertEqual((moved['before'], moved['after']), (WIND, None))

    def test_same_port_other_ip_counts_as_ours(self):
        cur = [{'port': 'C', 'endpoint': 'udp:192.168.2.1:27002'}]
        new, changes = bs.plan_serial_changes(cur, None, 7, WIND, GPS)
        self.assertEqual(new, [{'port': 'H', 'endpoint': GPS}])
        self.assertIn(('C', 'moved-from'), kinds(changes))

    def test_wind_only_leaves_gps_letter_untouched(self):
        cur = [{'port': 'H', 'endpoint': '/dev/ttyAMA4'}]
        new, changes = bs.plan_serial_changes(cur, 6, None, WIND, GPS)
        self.assertEqual(by_letter(new)['H'], '/dev/ttyAMA4')
        self.assertEqual({c['letter'] for c in changes}, {'G'})

    def test_preserves_order_and_unrelated_entries(self):
        new, _ = bs.plan_serial_changes(LIVE, 6, 7, WIND, GPS)
        self.assertEqual([e['port'] for e in new], ['D', 'E', 'B', 'G', 'H'])
        self.assertEqual(by_letter(new)['D'], 'udpin:0.0.0.0:27000')

    def test_never_touches_letter_a(self):
        cur = [{'port': 'A', 'endpoint': 'udp:127.0.0.1:27001'}]
        new, changes = bs.plan_serial_changes(cur, 6, None, WIND, GPS)
        self.assertEqual(by_letter(new)['A'], 'udp:127.0.0.1:27001')
        self.assertNotIn('A', {c['letter'] for c in changes})


class UndoPlanTests(unittest.TestCase):
    def test_restores_before_when_current_matches_after(self):
        snap = {'H': {'before': '/dev/ttyAMA4', 'after': GPS, 'route': 'gps'}}
        new, rows = bs.plan_serial_undo([{'port': 'H', 'endpoint': GPS}], snap,
                                        exists=lambda p: True)
        self.assertEqual(new, [{'port': 'H', 'endpoint': '/dev/ttyAMA4'}])
        self.assertEqual(rows[0]['kind'], 'restore')

    def test_removes_entry_that_was_newly_set(self):
        cur = LIVE + [{'port': 'H', 'endpoint': GPS}]
        snap = {'H': {'before': None, 'after': GPS, 'route': 'gps'}}
        new, rows = bs.plan_serial_undo(cur, snap)
        self.assertEqual(new, LIVE)
        self.assertEqual(rows[0]['kind'], 'remove')

    def test_puts_moved_entry_back(self):
        new_after_setup, changes = bs.plan_serial_changes(LIVE, 5, None, WIND, GPS)
        snap = {c['letter']: {'before': c['before'], 'after': c['after'],
                              'route': c['route']}
                for c in changes if c['kind'] != 'unchanged'}
        new, rows = bs.plan_serial_undo(new_after_setup, snap)
        self.assertEqual(by_letter(new), by_letter(LIVE))

    def test_skips_letter_changed_since_setup(self):
        snap = {'H': {'before': None, 'after': GPS, 'route': 'gps'}}
        cur = [{'port': 'H', 'endpoint': '/dev/ttyAMA9'}]
        new, rows = bs.plan_serial_undo(cur, snap)
        self.assertEqual(new, cur)
        self.assertEqual(rows[0]['kind'], 'skipped_changed')

    def test_skips_missing_device_before(self):
        snap = {'H': {'before': '/dev/ttyUSB7', 'after': GPS, 'route': 'gps'}}
        cur = [{'port': 'H', 'endpoint': GPS}]
        new, rows = bs.plan_serial_undo(cur, snap, exists=lambda p: False)
        self.assertEqual(new, cur)
        self.assertEqual(rows[0]['kind'], 'skipped_missing_device')


class _Resp:
    def __init__(self, status_code=200, body=None, text=None):
        self.status_code = status_code
        self._body = body
        self.text = text if text is not None else json.dumps(body)

    def json(self):
        return json.loads(self.text)


class _Session:
    def __init__(self, get=None, put=None, post=None, raise_on=()):
        self.get_resp, self.put_resp, self.post_resp = get or {}, put, post
        self.raise_on = raise_on
        self.calls = []

    def _maybe_raise(self, method, exc_msg='connection refused'):
        if method in self.raise_on:
            raise RuntimeError(exc_msg)

    def get(self, url, timeout=None):
        self.calls.append(('GET', url, None, timeout))
        self._maybe_raise('get')
        for suffix, r in self.get_resp.items():
            if url.endswith(suffix):
                return r
        return _Resp(404, text='')

    def put(self, url, json=None, timeout=None):
        self.calls.append(('PUT', url, json, timeout))
        self._maybe_raise('put')
        return self.put_resp

    def post(self, url, json=None, timeout=None):
        self.calls.append(('POST', url, json, timeout))
        self._maybe_raise('post')
        if isinstance(self.post_resp, Exception):
            raise self.post_resp
        return self.post_resp


class ClientTests(unittest.TestCase):
    def client(self, session):
        c = bs.SerialsClient(base_url='http://fake/ardupilot-manager/v1.0')
        c._session = session
        return c

    def test_get_board_and_serials(self):
        s = _Session(get={'/board': _Resp(body={'name': 'Navigator'}),
                          '/serials': _Resp(body=LIVE + [{'port': '', 'endpoint': 'x'}])})
        c = self.client(s)
        self.assertEqual(c.get_board(), {'name': 'Navigator'})
        self.assertEqual(c.get_serials(), LIVE)

    def test_put_sends_full_list(self):
        s = _Session(put=_Resp(200, text='null'))
        ok, _ = self.client(s).put_serials(LIVE)
        self.assertTrue(ok)
        method, url, body, _ = s.calls[0]
        self.assertEqual((method, url), ('PUT', 'http://fake/ardupilot-manager/v1.0/serials'))
        self.assertEqual(body, LIVE)

    def test_put_422_returns_reason(self):
        s = _Session(put=_Resp(422, text='{"detail":[{"msg":"Invalid endpoint"}]}'))
        ok, reason = self.client(s).put_serials(LIVE)
        self.assertFalse(ok)
        self.assertIn('422', reason)
        self.assertIn('Invalid endpoint', reason)

    def test_client_never_raises(self):
        c = self.client(_Session(raise_on=('get', 'put', 'post')))
        self.assertIsNone(c.get_board())
        self.assertIsNone(c.get_serials())
        self.assertFalse(c.put_serials(LIVE)[0])
        self.assertFalse(c.restart()[0])

    def test_restart_uses_long_timeout_and_treats_timeout_as_maybe(self):
        s = _Session(post=_Resp(200, text='null'))
        c = self.client(s)
        self.assertEqual(c.restart(), (True, 'restarted'))
        self.assertGreaterEqual(s.calls[0][3], 60)

        class ReadTimeout(Exception):
            pass
        s.post_resp = ReadTimeout('read timed out')
        self.assertEqual(c.restart(), (True, 'timeout'))


if __name__ == '__main__':
    unittest.main()
