"""Dual-route UDP streaming tests.

These verify:
  * MWV goes only to the wind port (27001)
  * GGA/RMC/VTG/HDT go only to the GPS port (27002)
  * Unrelated sentences (MDA, ROT, etc.) do not go anywhere
  * Per-route counters and the legacy total counter agree
  * Talker-prefixed forms ('WIMWV', 'GNGGA') still route correctly

Uses NMEAHandler directly with a fake UDP socket so nothing hits the wire.
No pytest dependency; use `python -m unittest`.
"""

from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from tests import _stubs
_stubs.install()


def _load_main_with_tempdirs():
    """Import app.main with the log dir redirected to a tempdir.

    NMEAHandler starts an auto-connect thread and a websocket/publisher
    thread; both are stubbed out. We also make sure the module is re-imported
    from scratch so state is clean between test runs.
    """
    repo_root = Path(__file__).resolve().parent.parent
    app_dir = repo_root / 'app'
    if str(app_dir) not in sys.path:
        sys.path.insert(0, str(app_dir))

    tmpdir = tempfile.TemporaryDirectory()
    log_dir = Path(tmpdir.name) / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    # main.py reads AIRMAR_WX_LOG_DIR so /app/logs is not required.
    import os
    os.environ['AIRMAR_WX_LOG_DIR'] = str(log_dir)
    sys.modules.pop('main', None)
    # Block background threads (auto-connect, websocket server, publisher).
    with mock.patch('threading.Thread.start', lambda self: None):
        import main as _main
    # Keep the tmpdir alive for the module's lifetime.
    _main._test_tmpdir = tmpdir  # type: ignore[attr-defined]
    return _main


class _FakeSocket:
    """Records `sendto((data, addr))` and never touches the network."""

    def __init__(self, to_receive=None):
        self.sent = []
        self.closed = False
        self.bound = None
        # Datagrams the "autopilot" has queued back to our sender.
        self.to_receive = list(to_receive or [])
        self.recv_calls = 0

    def bind(self, addr):
        self.bound = addr

    def recv(self, bufsize, flags=0):
        self.recv_calls += 1
        if not self.to_receive:
            raise BlockingIOError('no data')
        return self.to_receive.pop(0)

    def sendto(self, data, addr):
        self.sent.append((data, addr))

    def close(self):
        self.closed = True


class UdpRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main = _load_main_with_tempdirs()
        cls.handler = cls.main.nmea_handler

    def setUp(self):
        # Reset counters and captured payloads between tests.
        self.handler.udp_socket = _FakeSocket()
        self.handler.is_streaming = True
        self.handler.streamed_messages = 0
        self.handler.streamed_wind_messages = 0
        self.handler.streamed_gps_messages = 0
        # Suppress the ~1 Hz drain unless a test opts in, so send-path
        # assertions aren't perturbed by recv() bookkeeping.
        self.handler._last_udp_drain_ts = time.monotonic()

    def test_mwv_routes_to_wind_port_only(self):
        self.handler.stream_message('$WIMWV,45.0,R,10.0,N,A*3B', 'MWV')
        sent = self.handler.udp_socket.sent
        self.assertEqual(len(sent), 1)
        payload, addr = sent[0]
        # Regression: this MUST be 127.0.0.1, not host.docker.internal.
        # ArduPilot's `udpin` socket connect()s to the sender address on
        # the first datagram; the docker bridge gateway sender address
        # (172.18.0.1) was rejected on subsequent datagrams because the
        # host's LAN address was pinned by connect(). Localhost keeps the
        # sender address stable and works because the container is
        # NetworkMode=host. See main.py:UDP_HOST for the full write-up.
        self.assertEqual(addr, ('127.0.0.1', 27001))
        self.assertTrue(payload.endswith(b'\n'))
        self.assertEqual(self.handler.streamed_wind_messages, 1)
        self.assertEqual(self.handler.streamed_gps_messages, 0)
        self.assertEqual(self.handler.streamed_messages, 1)

    def test_gga_routes_to_gps_port_only(self):
        self.handler.stream_message('$GPGGA,...', 'GGA')
        sent = self.handler.udp_socket.sent
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][1], ('127.0.0.1', 27002))
        self.assertEqual(self.handler.streamed_gps_messages, 1)
        self.assertEqual(self.handler.streamed_wind_messages, 0)

    def test_gps_sentence_family_all_go_to_gps_port(self):
        for msg_type, raw in [
            ('GGA', '$GPGGA,...'),
            ('RMC', '$GPRMC,...'),
            ('VTG', '$GPVTG,...'),
            ('HDT', '$HCHDT,...'),
        ]:
            self.handler.stream_message(raw, msg_type)
        for _payload, addr in self.handler.udp_socket.sent:
            self.assertEqual(addr[1], 27002)
        self.assertEqual(self.handler.streamed_gps_messages, 4)
        self.assertEqual(self.handler.streamed_wind_messages, 0)

    def test_talker_prefixed_forms_route_correctly(self):
        # `_read_serial_loop` normalizes talker prefixes, but stream_message
        # is defensive and matches the trailing 3-char sentence code.
        self.handler.stream_message('$WIMWV,...', 'WIMWV')
        self.handler.stream_message('$GNGGA,...', 'GNGGA')
        endpoints = [addr[1] for _p, addr in self.handler.udp_socket.sent]
        self.assertEqual(endpoints, [27001, 27002])

    def test_unrelated_sentences_are_not_forwarded(self):
        for msg_type in ['MDA', 'ROT', 'XDR', 'ZDA', 'HDG']:
            self.handler.stream_message(f'$WI{msg_type},...', msg_type)
        self.assertEqual(self.handler.udp_socket.sent, [])
        self.assertEqual(self.handler.streamed_messages, 0)

    def test_not_streaming_sends_nothing(self):
        self.handler.is_streaming = False
        try:
            self.handler.stream_message('$WIMWV,...', 'MWV')
            self.assertEqual(self.handler.udp_socket.sent, [])
        finally:
            self.handler.is_streaming = True

    def test_get_stream_routes_reports_both_ports(self):
        self.handler.streamed_wind_messages = 3
        self.handler.streamed_gps_messages = 7
        routes = self.handler.get_stream_routes()
        self.assertEqual(len(routes), 2)
        by_name = {r['name']: r for r in routes}
        self.assertEqual(by_name['wind']['port'], 27001)
        self.assertEqual(by_name['gps']['port'], 27002)
        self.assertEqual(by_name['wind']['streamed_messages'], 3)
        self.assertEqual(by_name['gps']['streamed_messages'], 7)
        self.assertEqual(sorted(by_name['wind']['sentences']), ['MWV'])
        self.assertEqual(sorted(by_name['gps']['sentences']),
                         ['GGA', 'HDT', 'RMC', 'VTG'])
        self.assertEqual(by_name['wind']['source_endpoint'],
                         '127.0.0.1:27100')
        self.assertEqual(by_name['gps']['source_endpoint'],
                         '127.0.0.1:27100')

    def test_udp_sender_binds_stable_source_port(self):
        """ArduPilot pins both source IP and source port after the first
        datagram. A new ephemeral port after an extension restart would
        leave both udpin receivers connected to the dead process."""
        fake = _FakeSocket()
        with mock.patch.object(self.main.socket, 'socket',
                               return_value=fake):
            created = self.handler._create_udp_socket()
        self.assertIs(created, fake)
        self.assertEqual(fake.bound, ('127.0.0.1', 27100))

    def test_udp_bind_failure_closes_socket(self):
        class _FailingBindSocket(_FakeSocket):
            def bind(self, addr):
                raise OSError('source port already in use')

        fake = _FailingBindSocket()
        with mock.patch.object(self.main.socket, 'socket',
                               return_value=fake):
            with self.assertRaises(OSError):
                self.handler._create_udp_socket()
        self.assertTrue(fake.closed)

    def test_drain_discards_autopilot_replies(self):
        """ArduPilot's NMEA GPS driver probes its connected peer, which
        is now our fixed sender port. Those datagrams must be consumed
        so the receive buffer doesn't just fill and stay full."""
        probes = [b'$PUBX,41,1,0023,0001,230400,0*1E\r\n',
                  b'CONFIG COM1 230400 8 n 1\r\n']
        sock = _FakeSocket(to_receive=list(probes))
        self.handler.udp_socket = sock
        self.handler._last_udp_drain_ts = 0.0   # force the drain to run
        self.handler.stream_message('$GPGGA,...', 'GGA')
        self.assertEqual(sock.to_receive, [], 'queued replies must be drained')
        # The sentence still went out.
        self.assertEqual(len(sock.sent), 1)
        self.assertEqual(sock.sent[0][1], ('127.0.0.1', 27002))

    def test_drain_is_throttled_not_per_message(self):
        """At ~40 Hz a per-send drain would be pure overhead; the drain
        runs about once a second."""
        sock = _FakeSocket()
        self.handler.udp_socket = sock
        self.handler._last_udp_drain_ts = 0.0
        self.handler.stream_message('$GPGGA,...', 'GGA')
        after_first = sock.recv_calls
        self.assertGreater(after_first, 0, 'first send should drain')
        for _ in range(20):
            self.handler.stream_message('$GPGGA,...', 'GGA')
        self.assertEqual(
            sock.recv_calls, after_first,
            'subsequent sends within the interval must not re-drain',
        )

    def test_drain_stops_at_bound(self):
        """A flood must not stall the serial reader thread."""
        flood = [b'x'] * (self.handler.UDP_DRAIN_MAX_DATAGRAMS + 50)
        sock = _FakeSocket(to_receive=flood)
        self.handler.udp_socket = sock
        self.handler._last_udp_drain_ts = 0.0
        self.handler.stream_message('$GPGGA,...', 'GGA')
        self.assertEqual(sock.recv_calls, self.handler.UDP_DRAIN_MAX_DATAGRAMS)
        self.assertEqual(len(sock.to_receive), 50)

    def test_drain_survives_socket_errors(self):
        """A torn-down socket mid-drain must not escape into the send
        path's error handler and trigger a needless socket rebuild."""
        class _ExplodingSocket(_FakeSocket):
            def recv(self, bufsize, flags=0):
                raise OSError('socket closed')

        sock = _ExplodingSocket()
        self.handler.udp_socket = sock
        self.handler._last_udp_drain_ts = 0.0
        self.handler.stream_message('$GPGGA,...', 'GGA')
        # Send succeeded and the socket was not swapped out.
        self.assertEqual(len(sock.sent), 1)
        self.assertIs(self.handler.udp_socket, sock)
        self.assertFalse(sock.closed)

    def test_status_snapshot_has_no_legacy_mode_key(self):
        snap = self.handler._stream_status_snapshot()
        self.assertNotIn('autopilot_mode', snap)
        self.assertNotIn('streaming_to', snap)  # replaced by `routes`
        self.assertIn('routes', snap)
        self.assertIn('streamed_wind_messages', snap)
        self.assertIn('streamed_gps_messages', snap)


if __name__ == '__main__':
    unittest.main()
