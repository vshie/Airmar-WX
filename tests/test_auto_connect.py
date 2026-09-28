"""Startup auto-connect and UI connect tests.

These verify:
  * With no saved port, auto-connect never probes any port (a GPS on
    another ttyUSB must not be opened or sent Airmar commands)
  * A saved by-id name that is no longer present is not replaced by the
    old ttyUSBn path (which may now be a different device)
  * A UI connect is not aborted by a cancel flag left over from an
    earlier cancelled auto-connect
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import _stubs
_stubs.install()

from tests.test_udp_routing import _load_main_with_tempdirs


class AutoConnectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main = _load_main_with_tempdirs()
        cls.handler = cls.main.nmea_handler

    def setUp(self):
        self.handler.state['port'] = None
        self.handler.state['port_id'] = None
        self.handler._cancel_connect = False
        self.calls = []

        def fake_connect(port, baud_rate=None, stay_at_4800=False, max_attempts=6):
            self.calls.append((port, baud_rate, self.handler._cancel_connect))
            return True, 'ok'

        patcher = mock.patch.object(self.handler, 'connect_serial', side_effect=fake_connect)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_no_saved_port_does_not_scan(self):
        with mock.patch.object(self.handler, 'get_ports',
                               return_value=['/dev/ttyUSB0', '/dev/ttyUSB1']):
            self.handler._auto_connect()
        self.assertEqual(self.calls, [])
        self.assertEqual(self.handler.connection_status,
                         self.handler.CONN_STATUS_DISCONNECTED)

    def test_saved_port_is_tried(self):
        with tempfile.NamedTemporaryFile() as f:
            self.handler.state['port'] = f.name
            self.handler._auto_connect()
        self.assertEqual([c[0] for c in self.calls], [f.name])

    def test_missing_by_id_does_not_fall_back_to_tty_path(self):
        with tempfile.NamedTemporaryFile() as f:
            # Legacy path exists but belongs to whatever enumerated there now.
            self.handler.state['port'] = f.name
            self.handler.state['port_id'] = 'usb-FTDI_not_plugged_in-if00-port0'
            self.handler._auto_connect()
        self.assertEqual(self.calls, [])

    def test_user_connect_clears_stale_cancel(self):
        # Simulates: auto-connect was cancelled from the UI, then the user
        # picked a port and clicked Connect.
        self.handler._cancel_connect = True
        ok, _ = self.handler.user_connect('/dev/ttyUSB1')
        self.assertTrue(ok)
        self.assertEqual(len(self.calls), 1)
        port, baud, cancel_flag = self.calls[0]
        self.assertEqual(port, '/dev/ttyUSB1')
        self.assertEqual(baud, 4800)
        self.assertFalse(cancel_flag)

    def test_user_connect_waits_for_in_flight_attempt(self):
        self.handler._connect_lock.acquire()
        try:
            ok, msg = self.handler.user_connect('/dev/ttyUSB1', lock_timeout=0.05)
        finally:
            self.handler._connect_lock.release()
        self.assertFalse(ok)
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main()
