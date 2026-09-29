"""One-click setup / undo: BlueOS serial list + ArduRover params + restart.

Runs the job bodies synchronously (`run_async=False`) against a fake
BlueOS ardupilot-manager and a fake autopilot so the whole flow, including
the persisted undo snapshot, is exercised without threads or HTTP.
"""

import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from tests import _stubs
_stubs.install()

from tests.test_state_migration import _load_main  # noqa: E402

WIND = 'udpin:0.0.0.0:27001'
GPS = 'udpin:0.0.0.0:27002'
LIVE = [
    {'port': 'D', 'endpoint': 'udpin:0.0.0.0:27000'},
    {'port': 'G', 'endpoint': WIND},
]


class FakeSerials:
    """Stands in for blueos_serials.SerialsClient."""

    def __init__(self, serials, board='Navigator', put_ok=True):
        self.serials = [dict(e) for e in serials]
        self.board = board
        self.put_ok = put_ok
        self.puts = []
        self.restarts = 0
        self.log = None  # shared event log, set by the test

    def get_board(self):
        return {'name': self.board}

    def get_serials(self):
        return [dict(e) for e in self.serials]

    def put_serials(self, entries):
        self.puts.append([dict(e) for e in entries])
        if self.log is not None:
            self.log.append('put')
        if not self.put_ok:
            return False, 'BlueOS rejected the serial list (HTTP 422): bad'
        self.serials = [dict(e) for e in entries]
        return True, 'saved'

    def restart(self):
        self.restarts += 1
        if self.log is not None:
            self.log.append('restart')
        return True, 'restarted'


class FakeAutopilot:
    """Stands in for mavlink_params.ParamClient against a param table."""

    def __init__(self, params, armed=False, readable=None, write_fails=()):
        self.params = dict(params)
        self.armed = armed
        self.readable = readable  # None = all readable, else a set
        self.write_fails = set(write_fails)
        self.writes = []
        self.log = None

    def _can_read(self, name):
        return name in self.params and (self.readable is None or name in self.readable)

    def is_armed(self):
        return self.armed

    def wait_for_fresh_heartbeat(self, timeout_s=120.0):
        return True

    def read(self, name, timeout_s=3.0):
        return float(self.params[name]) if self._can_read(name) else None

    def write(self, name, value):
        if name in self.write_fails:
            return False, 'mavlink2rest rejected PARAM_SET', False
        self.writes.append((name, float(value)))
        if self.log is not None:
            self.log.append('write')
        self.params[name] = float(value)
        return True, 'verified via PARAM_VALUE echo', True

    def read_expected(self, expected):
        out = {}
        for name, target in expected.items():
            cur = self.read(name)
            out[name] = {'target': float(target), 'current': cur,
                         'available': cur is not None, 'resolved_name': name,
                         'match': cur is not None and abs(cur - float(target)) < 1e-4,
                         'reason': ''}
        return out

    def apply_expected(self, expected):
        out = {}
        for name, target in expected.items():
            prev = self.read(name)
            row = {'target': float(target), 'previous': prev, 'current': prev,
                   'resolved_name': name, 'available': prev is not None,
                   'reason': ''}
            if prev is not None and abs(prev - float(target)) < 1e-4:
                row.update(action='noop', ok=True)
            else:
                posted, why, _ = self.write(name, target)
                row.update(action='wrote' if posted else 'failed', ok=posted,
                           reason='' if posted else why,
                           current=float(target) if posted else prev)
            out[name] = row
        return out


BASE_PARAMS = {
    'SERIAL6_PROTOCOL': 21, 'SERIAL7_PROTOCOL': 2, 'WNDVN_TYPE': 0,
    'WNDVN_SPEED_TYPE': 0, 'GPS1_TYPE': 1, 'GPS2_TYPE': 0,
    'EK3_SRC2_YAW': 0, 'EK3_SRC2_POSXY': 0, 'EK3_SRC2_VELXY': 0,
}


class FlowTestBase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        log_dir = Path(tmp.name) / 'logs'
        log_dir.mkdir()
        main = _load_main(log_dir)
        self.h = main.nmea_handler
        self.h.state_path = Path(tmp.name) / 'state.json'
        self.h.state['autopilot_setup'] = {}
        self.h._setup_settle_s = 0
        self.serials = FakeSerials(LIVE)
        self.ap = FakeAutopilot(BASE_PARAMS)
        self.h._serials_client = self.serials
        self.h._param_client = self.ap
        # No /dev entries in these lists; keep the real FS out of it.
        patcher = mock.patch('os.path.exists', return_value=True)
        patcher.start()
        self.addCleanup(patcher.stop)

    def preview(self, wind=6, gps=7, yaw=False):
        ok, msg, payload = self.h.preview_serial_setup(wind, gps, yaw)
        self.assertTrue(ok, msg)
        return payload['preview']

    def confirm(self, preview, wind=6, gps=7, yaw=False, drop=False):
        ok, msg, payload = self.h.start_setup_job(
            'setup', preview['token'], drop, wind, gps, yaw, run_async=False)
        self.assertTrue(ok, msg)
        return self.h.get_setup_job()

    def undo(self, drop=False):
        ok, msg, payload = self.h.preview_serial_undo()
        self.assertTrue(ok, msg)
        ok, msg, _ = self.h.start_setup_job(
            'undo', payload['preview']['token'], drop, run_async=False)
        self.assertTrue(ok, msg)
        return self.h.get_setup_job()

    def saved(self):
        return json.loads(self.h.state_path.read_text())['autopilot_setup']


class SetupFlowTests(FlowTestBase):
    def test_preview_reports_serial_and_param_changes(self):
        p = self.preview()
        kinds = {(c['letter'], c['kind']) for c in p['serial_changes']}
        self.assertEqual(kinds, {('G', 'unchanged'), ('H', 'set')})
        params = {r['name']: r for r in p['param_changes']}
        self.assertEqual(params['SERIAL7_PROTOCOL']['before'], 2)
        self.assertEqual(params['SERIAL7_PROTOCOL']['after'], 5)
        self.assertTrue(params['SERIAL6_PROTOCOL']['unchanged'])
        self.assertFalse(p['armed'])
        self.assertTrue(p['will_restart'])
        self.assertEqual(self.serials.puts, [], 'preview must not write')
        self.assertEqual(self.ap.writes, [])

    def test_confirm_writes_serials_params_and_restarts_once(self):
        job = self.confirm(self.preview())
        self.assertTrue(job['ok'], job['message'])
        self.assertEqual(self.serials.serials, LIVE + [{'port': 'H', 'endpoint': GPS}])
        self.assertEqual(self.serials.restarts, 1)
        self.assertEqual(self.ap.params['SERIAL7_PROTOCOL'], 5)
        self.assertEqual(self.ap.params['GPS2_TYPE'], 5)
        saved = self.saved()
        self.assertTrue(saved['applied'])
        self.assertIn('serial_auto_applied', saved)

    def test_serials_written_before_params_and_restart_last(self):
        log = []
        self.serials.log = self.ap.log = log
        self.confirm(self.preview())
        self.assertEqual(log[0], 'put')
        self.assertEqual(log[-1], 'restart')

    def test_confirm_refuses_when_armed(self):
        p = self.preview()
        self.ap.armed = True
        job = self.confirm(p)
        self.assertFalse(job['ok'])
        self.assertIn('armed', job['message'])
        self.assertEqual(self.serials.puts, [])
        self.assertEqual(self.ap.writes, [])
        self.assertEqual(self.serials.restarts, 0)

    def test_confirm_refuses_without_heartbeat(self):
        p = self.preview()
        self.ap.armed = None
        job = self.confirm(p)
        self.assertFalse(job['ok'])
        self.assertEqual(self.serials.puts, [])

    def test_confirm_rejects_stale_token(self):
        p = self.preview()
        self.serials.serials.append({'port': 'C', 'endpoint': 'udpin:0.0.0.0:14550'})
        job = self.confirm(p)
        self.assertFalse(job['ok'])
        self.assertIn('changed since the preview', job['message'])
        self.assertEqual(self.serials.puts, [])

    def test_snapshot_persisted_before_put(self):
        seen = {}
        real_put = self.serials.put_serials

        def spy(entries):
            seen['undo'] = json.loads(self.h.state_path.read_text())[
                'autopilot_setup'].get('undo')
            return real_put(entries)
        self.serials.put_serials = spy
        self.confirm(self.preview())
        self.assertIsNotNone(seen['undo'])
        self.assertEqual(seen['undo']['serial_changes']['H'],
                         {'before': None, 'after': GPS, 'route': 'gps'})
        self.assertEqual(seen['undo']['params_before']['SERIAL7_PROTOCOL']['value'], 2)

    def test_put_422_aborts_without_param_writes_or_restart(self):
        self.serials.put_ok = False
        job = self.confirm(self.preview())
        self.assertFalse(job['ok'])
        self.assertIn('422', job['message'])
        self.assertEqual(self.ap.writes, [])
        self.assertEqual(self.serials.restarts, 0)
        self.assertNotIn('undo', self.saved())

    def test_all_param_failures_roll_back_serials_without_restart(self):
        self.ap.write_fails = set(BASE_PARAMS)
        job = self.confirm(self.preview())
        self.assertFalse(job['ok'])
        self.assertEqual(self.serials.serials, LIVE)
        self.assertEqual(self.serials.restarts, 0)
        self.assertNotIn('undo', self.saved())

    def test_restart_skipped_when_nothing_changed(self):
        self.confirm(self.preview())
        self.serials.restarts = 0
        job = self.confirm(self.preview())
        self.assertTrue(job['ok'], job['message'])
        self.assertEqual(self.serials.restarts, 0)

    def test_reapply_keeps_original_befores(self):
        self.confirm(self.preview())
        # Re-run with wind moved to SERIAL5: new letters join the snapshot,
        # but what was there before the first setup is kept.
        self.confirm(self.preview(wind=5), wind=5)
        undo = self.saved()['undo']
        self.assertEqual(undo['serial_changes']['H']['before'], None)
        self.assertEqual(undo['serial_changes']['G']['before'], WIND)
        self.assertEqual(undo['serial_changes']['F']['before'], None)
        self.assertEqual(undo['params_before']['SERIAL7_PROTOCOL']['value'], 2)

    def test_dead_device_blocks_until_dropped(self):
        self.serials.serials.append({'port': 'E', 'endpoint': '/dev/ttyUSB9'})
        with mock.patch('os.path.exists', side_effect=lambda p: p != '/dev/ttyUSB9'):
            p = self.preview()
            self.assertEqual(p['dead_devices'], [{'port': 'E', 'endpoint': '/dev/ttyUSB9'}])
            job = self.confirm(p)
            self.assertFalse(job['ok'])
            self.assertEqual(self.serials.puts, [])
            job = self.confirm(self.preview(), drop=True)
        self.assertTrue(job['ok'], job['message'])
        self.assertNotIn('E', {e['port'] for e in self.serials.serials})

    def test_unsupported_board_preview_fails(self):
        self.serials.board = 'Pixhawk1'
        ok, msg, _ = self.h.preview_serial_setup(6, 7, False)
        self.assertFalse(ok)
        self.assertIn('unavailable', msg)

    def test_serial_8_rejected(self):
        ok, msg, _ = self.h.preview_serial_setup(8, None, False)
        self.assertFalse(ok)
        self.assertIn('SERIAL8', msg)

    def test_second_job_rejected_at_once_while_a_job_runs(self):
        self.h.SETUP_LOCK_WAIT_S = 5
        self.h.state['autopilot_setup'] = {'applied': True, 'expected': {'GPS2_TYPE': 5.0}}
        self.h._autopilot_setup_lock.acquire()
        self.h._setup_job = {'id': 1, 'running': True}
        try:
            t0 = time.monotonic()
            ok, msg, _ = self.h.start_setup_job('undo', 'x', run_async=False)
            self.assertFalse(ok)
            self.assertIn('in progress', msg)
            ok, msg, _ = self.h.check_autopilot_setup()
            self.assertFalse(ok)
            self.assertLess(time.monotonic() - t0, 1, 'must not wait out a job')
        finally:
            self.h._autopilot_setup_lock.release()

    def test_confirm_waits_for_a_running_check(self):
        # Regression (1.1.1 on the vehicle): the UI's 30 s background Check
        # held the lock and a Confirm clicked meanwhile was refused.
        p = self.preview()
        self.h._autopilot_setup_lock.acquire()
        threading.Timer(0.2, self.h._autopilot_setup_lock.release).start()
        job = self.confirm(p)
        self.assertTrue(job['ok'], job['message'])
        self.assertEqual(self.serials.restarts, 1)

    def test_short_operations_give_up_after_the_wait(self):
        self.h.SETUP_LOCK_WAIT_S = 0.05
        self.h._autopilot_setup_lock.acquire()
        try:
            ok, msg, _ = self.h.preview_serial_setup(6, 7, False)
            self.assertFalse(ok)
            self.assertIn('in progress', msg)
        finally:
            self.h._autopilot_setup_lock.release()

    def test_status_includes_serial_auto_and_job_without_http(self):
        self.h._serials_client = None  # any HTTP attempt would crash
        self.h._serials_client = type('X', (), {})()
        status = self.h.get_autopilot_setup_status()
        self.assertIn('serial_auto', status)
        self.assertIn('job', status)
        self.assertFalse(status['undo_available'])


class UndoFlowTests(FlowTestBase):
    def test_undo_restores_serials_and_params_then_clears_snapshot(self):
        self.confirm(self.preview())
        self.serials.restarts = 0
        job = self.undo()
        self.assertTrue(job['ok'], job['message'])
        self.assertEqual(self.serials.serials, LIVE)
        self.assertEqual(self.serials.restarts, 1)
        for name, value in BASE_PARAMS.items():
            self.assertEqual(self.ap.params[name], value, name)
        saved = self.saved()
        self.assertNotIn('undo', saved)
        self.assertFalse(saved['applied'])
        self.assertEqual(saved['expected'], {})
        self.assertNotIn('serial_auto_applied', saved)

    def test_undo_puts_moved_entry_back(self):
        self.confirm(self.preview(wind=5), wind=5)
        self.assertNotIn('G', {e['port'] for e in self.serials.serials})
        self.undo()
        self.assertEqual({e['port']: e['endpoint'] for e in self.serials.serials},
                         {e['port']: e['endpoint'] for e in LIVE})

    def test_undo_reports_unknown_param_originals(self):
        self.ap.readable = set(BASE_PARAMS) - {'GPS2_TYPE'}
        self.confirm(self.preview())
        self.ap.readable = None
        job = self.undo()
        rows = {r['name']: r for r in job['result']['param_changes']}
        self.assertEqual(rows['GPS2_TYPE']['action'], 'skipped_unknown')
        self.assertEqual(self.ap.params['GPS2_TYPE'], 5)  # left as written

    def test_undo_keeps_partial_snapshot_on_failure(self):
        self.confirm(self.preview())
        self.ap.write_fails = {'GPS2_TYPE'}
        job = self.undo()
        self.assertFalse(job['ok'])
        undo = self.saved()['undo']
        self.assertEqual(set(undo['params_before']), {'GPS2_TYPE'})
        self.assertEqual(undo['serial_changes'], {})

    def test_undo_leaves_letter_changed_since_setup(self):
        self.confirm(self.preview())
        self.serials.serials = [e if e['port'] != 'H' else
                                {'port': 'H', 'endpoint': 'udpin:0.0.0.0:14660'}
                                for e in self.serials.serials]
        job = self.undo()
        rows = {r['letter']: r for r in job['result']['serial_changes']}
        self.assertEqual(rows['H']['kind'], 'skipped_changed')
        self.assertIn({'port': 'H', 'endpoint': 'udpin:0.0.0.0:14660'},
                      self.serials.serials)


if __name__ == '__main__':
    unittest.main()
