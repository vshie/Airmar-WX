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


if __name__ == '__main__':
    unittest.main()
