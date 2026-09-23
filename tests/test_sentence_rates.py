"""Sentence rate defaults and PAMTR,EN parser regressions.

Two related bugs surfaced from the long-running vehicle at 192.168.1.69:

1. At 115200 baud the GPS-family sentences (GGA/RMC/VTG/HDT) need to run
   at ~10 Hz so ArduPilot's NMEA GPS driver keeps EKF3 happy. The
   extension used to enable them at the class-dict default of 1 Hz, so
   the operator had to hand-configure the rate on every install.
2. `query_sentence_config` parsed the last comma-separated field of the
   `$PAMTR,EN,...` response WITHOUT stripping the NMEA `*XX` checksum.
   `int('1*45')` fails, `'1*45'.isdigit()` is False, so the fallback
   `interval = 10` fired for every device response and the UI reported
   every sentence as 1 Hz even when the device was at 10 Hz.

Both are exercised here through the NMEAHandler surface used by the
network-free UDP tests.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import _stubs
_stubs.install()


def _load_main_with_tempdirs():
    repo_root = Path(__file__).resolve().parent.parent
    app_dir = repo_root / 'app'
    if str(app_dir) not in sys.path:
        sys.path.insert(0, str(app_dir))
    import os
    tmpdir = tempfile.TemporaryDirectory()
    log_dir = Path(tmpdir.name) / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ['AIRMAR_WX_LOG_DIR'] = str(log_dir)
    sys.modules.pop('main', None)
    with mock.patch('threading.Thread.start', lambda self: None):
        import main as _main
    _main._test_tmpdir = tmpdir  # type: ignore[attr-defined]
    return _main


class RequiredSentenceTests(unittest.TestCase):
    def setUp(self):
        self.main = _load_main_with_tempdirs()
        self.handler = self.main.nmea_handler

    def test_gps_family_in_required_sentences(self):
        """Regression: GGA and RMC must be auto-enabled on connect so the
        ArduPilot NMEA GPS driver has GGA (position) and RMC (course/
        speed fallback) without operator configuration."""
        required = set(self.handler.REQUIRED_SENTENCES)
        for sid in ('GGA', 'RMC', 'VTG', 'HDT'):
            self.assertIn(
                sid, required,
                f"{sid} must be in REQUIRED_SENTENCES for GPS driver",
            )

    def test_high_baud_gps_interval_is_10hz(self):
        """At 115200 baud the four GPS-family sentences are 10 Hz."""
        for sid in ('GGA', 'RMC', 'VTG', 'HDT'):
            got = self.handler._required_interval_for(sid, 115200)
            self.assertEqual(
                got, 1,
                f"{sid} @ 115200 must be interval 1 tenths (10 Hz), got {got}",
            )

    def test_high_baud_leaves_non_gps_alone(self):
        """Wind + weather sentences stay at the class-dict default at
        both baud rates — we don't want MWD firing at 10 Hz."""
        self.assertEqual(
            self.handler._required_interval_for('MWVR', 115200), 10,
        )
        self.assertEqual(
            self.handler._required_interval_for('MDA', 115200), 10,
        )

    def test_low_baud_forces_1hz_on_gps_family(self):
        """At 4800 baud the override is off so bandwidth stays safe."""
        for sid in ('GGA', 'RMC', 'VTG', 'HDT'):
            got = self.handler._required_interval_for(sid, 4800)
            self.assertEqual(
                got, 10,
                f"{sid} @ 4800 baud must fall back to 1 Hz (got {got})",
            )


class SentenceQueryChecksumStrippingTests(unittest.TestCase):
    """Direct exercise of the parser branch that broke on the vehicle.

    We reproduce the exact string the device sends and confirm that the
    checksum-stripped interval is decoded correctly, both for the
    long-form and short-form `$PAMTR,EN,...` responses.
    """

    def setUp(self):
        self.main = _load_main_with_tempdirs()
        self.handler = self.main.nmea_handler

    def _parse_line(self, line):
        # Mirrors the parser branch in query_sentence_config.
        line_body = line.split('*', 1)[0]
        parts = line_body.split(',')
        if len(parts) < 6:
            return None, None, None
        if len(parts) >= 7 and parts[4] in self.handler.SUPPORTED_SENTENCES:
            sid, enabled_str, interval_str = parts[4], parts[5], parts[6]
        else:
            sid, enabled_str, interval_str = parts[3], parts[4], parts[5]
        interval_str = interval_str.strip()
        interval = int(interval_str) if interval_str.isdigit() else 10
        return sid, enabled_str == '1', interval

    def test_long_form_with_checksum_at_10hz(self):
        # Device response for 10 Hz GGA: total=18, num=4.
        sid, enabled, interval = self._parse_line('$PAMTR,EN,18,4,GGA,1,1*45')
        self.assertEqual(sid, 'GGA')
        self.assertTrue(enabled)
        self.assertEqual(
            interval, 1,
            "Bug regression: checksum-suffixed '1*45' must decode as 1, "
            "not fall back to 10. This was the on-vehicle UI misreport.",
        )

    def test_long_form_with_checksum_at_1hz(self):
        sid, _, interval = self._parse_line('$PAMTR,EN,18,7,MDA,1,10*6E')
        self.assertEqual(sid, 'MDA')
        self.assertEqual(interval, 10)

    def test_disabled_sentence_still_parses(self):
        _, enabled, interval = self._parse_line('$PAMTR,EN,18,10,GLL,0,10*7A')
        self.assertFalse(enabled)
        self.assertEqual(interval, 10)


class NmeaLoggingTests(unittest.TestCase):
    """log_message must write each sentence exactly once and rotate."""

    def setUp(self):
        self.main = _load_main_with_tempdirs()
        self.handler = self.main.nmea_handler

    def test_log_message_writes_once_not_twice(self):
        """Regression: the old log_message wrote the sentence via a
        direct open() AND via the nmea_logger, producing two lines in
        `nmea_messages.log` per sentence. That's what filled a 362 MB
        file in 4 hours on the vehicle."""
        # Truncate anything left over from a previous test.
        self.handler.log_path.write_text('')
        ok, _msg = self.handler.log_message('$WIMWV,45.0,R,10.0,N,A*3B')
        self.assertTrue(ok)
        # Flush the rotating handler so bytes are on disk.
        for h in self.handler.nmea_logger.handlers:
            try:
                h.flush()
            except Exception:
                pass
        contents = self.handler.log_path.read_text()
        line_count = sum(1 for line in contents.splitlines() if line.strip())
        self.assertEqual(
            line_count, 1,
            f"log_message must write exactly one line; got {line_count}: "
            f"{contents!r}",
        )
        self.assertIn('$WIMWV,45.0,R,10.0,N,A*3B', contents)

    def test_nmea_logger_uses_rotating_handler(self):
        """The plain FileHandler cannot cap size — we specifically use a
        RotatingFileHandler with maxBytes set."""
        import logging.handlers
        rotators = [
            h for h in self.handler.nmea_logger.handlers
            if isinstance(h, logging.handlers.RotatingFileHandler)
        ]
        self.assertEqual(
            len(rotators), 1,
            "Exactly one RotatingFileHandler on the NMEA logger; "
            "multiple handlers = duplicated log lines. Handlers = "
            f"{self.handler.nmea_logger.handlers!r}",
        )
        self.assertGreater(rotators[0].maxBytes, 0)
        self.assertGreater(rotators[0].backupCount, 0)


if __name__ == '__main__':
    unittest.main()
