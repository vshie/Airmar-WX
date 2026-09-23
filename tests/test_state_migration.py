"""State migration: old `autopilot_mode` key must be dropped silently.

The extension used to persist `autopilot_mode` ('windvane' or 'gps'). The
dual-route rewrite removes that key entirely; loading a legacy state file
must not raise, must not resurrect the key, and must leave unrelated
settings alone.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import _stubs
_stubs.install()


def _load_main(log_dir):
    repo_root = Path(__file__).resolve().parent.parent
    app_dir = repo_root / 'app'
    if str(app_dir) not in sys.path:
        sys.path.insert(0, str(app_dir))
    import os
    os.environ['AIRMAR_WX_LOG_DIR'] = str(log_dir)
    sys.modules.pop('main', None)
    with mock.patch('threading.Thread.start', lambda self: None):
        import main as _main
    return _main


class MigrationTests(unittest.TestCase):
    def test_legacy_autopilot_mode_removed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_dir = Path(tmpdir) / 'logs'
            log_dir.mkdir()
            main = _load_main(log_dir)
            state_path = Path(tmpdir) / 'state.json'
            legacy = {
                'port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'stay_at_4800': False,
                'is_streaming': True,
                'autopilot_mode': 'windvane',   # legacy
                'sentence_config': {'MWVR': {'enabled': True, 'interval': 10}},
            }
            state_path.write_text(json.dumps(legacy))
            handler = main.nmea_handler
            handler.state_path = state_path
            # Reset to defaults so load_state's update path is exercised.
            handler.state = {
                'port': None, 'baud_rate': 4800, 'stay_at_4800': False,
                'is_streaming': False, 'sentence_config': {}, 'autopilot_setup': {},
            }
            handler.load_state()
            self.assertNotIn('autopilot_mode', handler.state)
            # Unrelated fields survive.
            self.assertEqual(handler.state['port'], '/dev/ttyUSB0')
            self.assertEqual(handler.state['baud_rate'], 115200)
            self.assertEqual(handler.state['sentence_config'],
                             {'MWVR': {'enabled': True, 'interval': 10}})
            # New setup slot exists.
            self.assertIsInstance(handler.state.get('autopilot_setup'), dict)

    def test_gps_contract_migration_from_gps1_nmea(self):
        """A persisted setup that wrote GPS1_TYPE=5 must be invalidated so
        the drift check surfaces the new two-GPS contract on next Check."""
        with tempfile.TemporaryDirectory() as tmpdir:
            log_dir = Path(tmpdir) / 'logs'
            log_dir.mkdir()
            main = _load_main(log_dir)
            state_path = Path(tmpdir) / 'state.json'
            legacy_setup = {
                'wind_serial': 6,
                'gps_serial': 7,
                'use_gps_yaw_fallback': False,
                'applied': True,
                'ignore_drift': True,
                'last_apply_ts': 1789956591.0,
                'expected': {
                    'SERIAL6_PROTOCOL': 21.0,
                    'WNDVN_TYPE': 4.0,
                    'WNDVN_SPEED_TYPE': 4.0,
                    'SERIAL7_PROTOCOL': 5.0,
                    'GPS1_TYPE': 5.0,     # legacy: NMEA on primary GPS
                    'EK3_SRC2_YAW': 2.0,
                    'EK3_SRC2_POSXY': 3.0,
                    'EK3_SRC2_VELXY': 3.0,
                },
                'last_apply_result': {'GPS1_TYPE': {'action': 'wrote', 'ok': True}},
            }
            state_path.write_text(json.dumps({
                'port': '/dev/ttyUSB0', 'baud_rate': 115200,
                'stay_at_4800': False, 'is_streaming': True,
                'sentence_config': {},
                'autopilot_setup': legacy_setup,
            }))
            handler = main.nmea_handler
            handler.state_path = state_path
            handler.state = {
                'port': None, 'baud_rate': 4800, 'stay_at_4800': False,
                'is_streaming': False, 'sentence_config': {}, 'autopilot_setup': {},
            }
            handler.load_state()

            setup = handler.state['autopilot_setup']
            # SERIAL selection preserved so the UI form is pre-populated.
            self.assertEqual(setup['wind_serial'], 6)
            self.assertEqual(setup['gps_serial'], 7)
            self.assertFalse(setup['use_gps_yaw_fallback'])
            # But the stale expected snapshot is dropped, apply flag
            # cleared, and ignore_drift reset so the operator is notified.
            self.assertEqual(setup['expected'], {})
            self.assertFalse(setup['applied'])
            self.assertFalse(setup['ignore_drift'])
            self.assertIsNone(setup['last_apply_ts'])
            # And the migrated state was persisted immediately.
            saved = json.loads(state_path.read_text())
            self.assertEqual(saved['autopilot_setup']['expected'], {})

    def test_gps_contract_migration_when_gps2_missing(self):
        """A pre-1.1.6 snapshot that lacks GPS2_TYPE (even if GPS1_TYPE
        was already something other than 5) must also be regenerated."""
        with tempfile.TemporaryDirectory() as tmpdir:
            log_dir = Path(tmpdir) / 'logs'
            log_dir.mkdir()
            main = _load_main(log_dir)
            state_path = Path(tmpdir) / 'state.json'
            state_path.write_text(json.dumps({
                'port': '/dev/ttyUSB0', 'baud_rate': 115200,
                'stay_at_4800': False, 'is_streaming': True,
                'sentence_config': {},
                'autopilot_setup': {
                    'wind_serial': 2, 'gps_serial': 3,
                    'use_gps_yaw_fallback': False,
                    'applied': True, 'ignore_drift': False,
                    'expected': {'SERIAL2_PROTOCOL': 21.0, 'GPS1_TYPE': 1.0},
                    'last_apply_result': {},
                },
            }))
            handler = main.nmea_handler
            handler.state_path = state_path
            handler.state = {
                'port': None, 'baud_rate': 4800, 'stay_at_4800': False,
                'is_streaming': False, 'sentence_config': {}, 'autopilot_setup': {},
            }
            handler.load_state()
            setup = handler.state['autopilot_setup']
            self.assertEqual(setup['expected'], {})
            self.assertFalse(setup['applied'])

    def test_gps_contract_migration_noop_for_new_snapshots(self):
        """Snapshots that already conform to the two-GPS contract must
        be left alone — no re-writes, no cleared ignore_drift."""
        with tempfile.TemporaryDirectory() as tmpdir:
            log_dir = Path(tmpdir) / 'logs'
            log_dir.mkdir()
            main = _load_main(log_dir)
            state_path = Path(tmpdir) / 'state.json'
            new_setup = {
                'wind_serial': 6, 'gps_serial': 7,
                'use_gps_yaw_fallback': False,
                'applied': True, 'ignore_drift': True,
                'expected': {
                    'SERIAL6_PROTOCOL': 21.0,
                    'WNDVN_TYPE': 4.0, 'WNDVN_SPEED_TYPE': 4.0,
                    'SERIAL7_PROTOCOL': 5.0,
                    'GPS1_TYPE': 1.0, 'GPS2_TYPE': 5.0,
                    'EK3_SRC2_YAW': 2.0,
                    'EK3_SRC2_POSXY': 3.0, 'EK3_SRC2_VELXY': 3.0,
                },
                'last_apply_ts': 1790000000.0,
                'last_apply_result': {},
            }
            state_path.write_text(json.dumps({
                'port': '/dev/ttyUSB0', 'baud_rate': 115200,
                'stay_at_4800': False, 'is_streaming': True,
                'sentence_config': {},
                'autopilot_setup': new_setup,
            }))
            handler = main.nmea_handler
            handler.state_path = state_path
            handler.state = {
                'port': None, 'baud_rate': 4800, 'stay_at_4800': False,
                'is_streaming': False, 'sentence_config': {}, 'autopilot_setup': {},
            }
            handler.load_state()
            setup = handler.state['autopilot_setup']
            self.assertEqual(setup['expected'], new_setup['expected'])
            self.assertTrue(setup['applied'])
            self.assertTrue(setup['ignore_drift'])


if __name__ == '__main__':
    unittest.main()
