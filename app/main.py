#!/usr/bin/env python3

import os
import serial
import serial.tools.list_ports
import logging
import logging.handlers
import json
import socket
import asyncio
from pathlib import Path
from flask import Flask, jsonify, request, send_file, send_from_directory, Response
from flask_cors import CORS
import datetime
import re
import threading
import time
import queue

try:
    import websockets
    from websockets.datastructures import Headers as WsHeaders
    from websockets.http11 import Response as WsResponse
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

try:
    from mavlink_sender import (
        DEFAULT_FRESH_S as _MAV_DEFAULT_FRESH_S,
        MavlinkSender,
        NAMED_VALUE_OFFSETS as _MAV_NVF_OFFSETS,
    )
    HAS_MAVLINK_SENDER = True
except ImportError:
    HAS_MAVLINK_SENDER = False

try:
    from mavlink_params import (
        ParamClient,
        MIN_SERIAL_INDEX as _MIN_SERIAL_INDEX,
        MAX_SERIAL_INDEX as _MAX_SERIAL_INDEX,
        build_expected_params,
        diff_current_vs_expected,
        snapshot_from_apply_result,
        validate_selection,
    )
    HAS_MAVLINK_PARAMS = True
except ImportError:
    HAS_MAVLINK_PARAMS = False

app = Flask(__name__, static_folder='static')
CORS(app)

class NMEAHandler:
    # Complete registry of WX-series supported NMEA sentences (200WX / 300WX, etc.)
    # Based on WX Series NMEA 0183 Technical Manual (300WXR, 300WX, 150WX, 100WX)
    # max_chars: worst-case sentence length from WX Series manual Table 1 (incl $, *hh, CR, LF = +7)
    SUPPORTED_SENTENCES = {
        'DTM':   {'name': 'Datum Reference', 'description': 'GPS datum reference', 'default_enabled': False, 'default_interval': 10, 'max_chars': 47},
        'GGA':   {'name': 'GPS Fix Data', 'description': 'Position, altitude, satellites, fix quality', 'default_enabled': True, 'default_interval': 10, 'max_chars': 82},
        'GLL':   {'name': 'Geographic Position', 'description': 'Latitude/Longitude position', 'default_enabled': False, 'default_interval': 10, 'max_chars': 48},
        'GSA':   {'name': 'DOP & Satellites', 'description': 'GPS dilution of precision and active satellites', 'default_enabled': False, 'default_interval': 10, 'max_chars': 66},
        'GSV':   {'name': 'Satellites in View', 'description': 'Satellite details (azimuth, elevation, SNR)', 'default_enabled': False, 'default_interval': 10, 'max_chars': 210},
        'HDG':   {'name': 'Heading (Magnetic)', 'description': 'Magnetic heading with deviation and variation', 'default_enabled': True, 'default_interval': 10, 'max_chars': 33},
        'HDT':   {'name': 'Heading (True)', 'description': 'True heading relative to north', 'default_enabled': False, 'default_interval': 10, 'max_chars': 18},
        'MDA':   {'name': 'Meteorological Composite', 'description': 'Pressure, temperature, humidity, dew point, wind', 'default_enabled': True, 'default_interval': 10, 'max_chars': 82},
        'MWD':   {'name': 'Wind Direction (True)', 'description': 'True wind direction and speed relative to north', 'default_enabled': False, 'default_interval': 10, 'max_chars': 55},
        'MWVR': {'name': 'Wind (Apparent)', 'description': 'Relative/apparent wind speed and angle', 'default_enabled': True, 'default_interval': 10, 'max_chars': 30},
        'MWVT': {'name': 'Wind (True)', 'description': 'True/theoretical wind speed and angle', 'default_enabled': False, 'default_interval': 10, 'max_chars': 30},
        'RMC':   {'name': 'Recommended Minimum', 'description': 'Position, speed, course, date, magnetic variation', 'default_enabled': True, 'default_interval': 10, 'max_chars': 72},
        'ROT':   {'name': 'Rate of Turn', 'description': 'Rate of turn in degrees per minute', 'default_enabled': False, 'default_interval': 10, 'max_chars': 18},
        'THS':   {'name': 'True Heading & Status', 'description': 'True heading with mode indicator', 'default_enabled': False, 'default_interval': 10, 'max_chars': 25},
        'VTG':   {'name': 'Course Over Ground', 'description': 'Course and speed over ground', 'default_enabled': False, 'default_interval': 10, 'max_chars': 42},
        'VWR':   {'name': 'Relative Wind (Alt)', 'description': 'Relative wind speed and angle (alternative format)', 'default_enabled': False, 'default_interval': 10, 'max_chars': 47},
        'VWT':   {'name': 'True Wind (Alt)', 'description': 'True wind speed and angle (alternative format)', 'default_enabled': True, 'default_interval': 10, 'max_chars': 47},
        'ZDA':   {'name': 'Time & Date', 'description': 'UTC time and date', 'default_enabled': False, 'default_interval': 10, 'max_chars': 38},
    }
    
    # Sentences auto-enabled on connection. Two groups:
    #   * Dashboard display (wind + heading + rate-of-turn + time/date).
    #   * ArduPilot GPS driver (`AP_GPS_NMEA`): GGA, RMC, VTG, HDT. These
    #     are the sentences the driver actually consumes; without GGA the
    #     driver reports "no fix" even when position is streaming, and
    #     without RMC there is no ground-truth speed/course fallback when
    #     VTG drops. They must also be fast enough that the EKF3 GPS
    #     branch stays healthy — see `_required_interval_for()`.
    REQUIRED_SENTENCES = ['MWVR', 'MWVT', 'MWD', 'HDT', 'VTG', 'ROT', 'ZDA',
                          'GGA', 'RMC']

    # PAMTC,EN interval override for the ArduPilot GPS sentences when we
    # are at the operating baud (115200). Value is tenths-of-seconds per
    # the WX manual, so `1` = 0.1 s = 10 Hz. ArduPilot's NMEA GPS driver
    # is happier at 5-10 Hz (EKF3 innovation rejects sluggish GPS), and
    # 4 sentences x 82 chars x 10 Hz = ~26 kbps, well under 115200 baud
    # with the wind + weather sentences at 1 Hz on the same bus.
    #
    # At 4800 baud we keep the class-dict default of 10 tenths (1 Hz) to
    # stay well under the ~4800 chars/s ceiling.
    HIGH_BAUD_GPS_HZ_OVERRIDE_TENTHS = {
        'GGA': 1, 'RMC': 1, 'VTG': 1, 'HDT': 1,
    }

    # ── ArduPilot UDP routes ─────────────────────────────────────────
    # Two always-active filtered streams. Each route sends only its own set
    # of NMEA sentences to a dedicated UDP port so a single ArduPilot serial
    # can be bound to each (BlueOS: `udpin:0.0.0.0:<port>` for SERIALx).
    #
    #   Wind route → 27001, drives AP_WindVane_NMEA (WNDVN_TYPE=4)
    #   GPS route  → 27002, drives AP_GPS NMEA driver (GPS2_TYPE=5) and yaw
    #
    # Historically this extension picked one mode at a time and shipped
    # everything to port 27000. That prevented feeding both drivers because
    # the wind driver only accepts $??MWV and the GPS driver only accepts
    # GGA/RMC/VTG/HDT. Two ports keep each driver clean and match the
    # per-serial protocol model in ArduPilot (SERIALx_PROTOCOL is one value).
    #
    # Host is 127.0.0.1: the container runs with NetworkMode=host, so the
    # host loopback is the autopilot's `udpin` bind address. Using
    # `host.docker.internal` (172.18.0.1, the docker bridge gateway) sent
    # datagrams from 192.168.2.12 (the host's LAN address). ArduPilot's
    # wildcard `udpin:0.0.0.0:PORT` socket accepts the first datagram, then
    # `UDPDevice::read()` calls `socket.connect()` on that sender's
    # (address, port) tuple. That pins the socket's local address to
    # 192.168.2.12, after which every subsequent datagram -- still
    # addressed to 172.18.0.1 -- silently no longer matches. ArduPilot
    # received exactly one datagram per boot and then went deaf, which is
    # why restarting never helped and the wind feed was silently broken in
    # the same way. Sending from 127.0.0.1 keeps the source address stable
    # for the lifetime of the process.
    UDP_HOST = '127.0.0.1'
    UDP_WIND_PORT = 27001
    UDP_GPS_PORT = 27002
    UDP_WIND_SENTENCES = frozenset({'MWV'})
    UDP_GPS_SENTENCES = frozenset({'GGA', 'RMC', 'VTG', 'HDT'})

    # After probe at 4800, switch to this rate (300WX supports up to 115200 via $PAMTC,BAUD).
    OPERATING_BAUD_RATE = 115200

    # 5-char addresses like $GNGGA = 2-char talker (GN) + 3-char sentence (GGA); normalize to sentence id.
    _NMEA_THREE_LETTER_SENTENCES = frozenset({
        'GGA', 'GLL', 'RMC', 'VTG', 'GSA', 'GSV', 'ZDA', 'DTM', 'GNS',
        'MWV', 'MWD', 'MDA', 'HDT', 'HDG', 'ROT', 'XDR', 'THS', 'VWR', 'VWT',
    })
    
    # Connection status phases
    CONN_STATUS_DISCONNECTED = 'disconnected'
    CONN_STATUS_AUTO_SCANNING = 'auto_scanning'
    CONN_STATUS_TRYING_4800 = 'trying_4800'
    CONN_STATUS_TRYING_HIGH_BAUD = 'trying_high_baud'
    CONN_STATUS_SWITCHING_BAUD = 'switching_baud'
    CONN_STATUS_ENABLING_SENTENCES = 'enabling_sentences'
    CONN_STATUS_CONNECTED = 'connected'
    CONN_STATUS_FAILED = 'failed'

    @staticmethod
    def _line_looks_like_nmea(line):
        """Heuristic shape check before checksum (probe / fast filter)."""
        if not line or len(line) < 7 or ',' not in line:
            return False
        return line.startswith('$') or line.startswith('!')

    @staticmethod
    def _nmea_cmd(payload: str) -> bytes:
        """Build $payload*hh\\r\\n — XOR checksum over payload (between $ and *), per NMEA 0183."""
        csum = 0
        for ch in payload:
            csum ^= ord(ch)
        return f'${payload}*{csum:02X}\r\n'.encode('ascii')

    @classmethod
    def _nmea_sentence_formatter(cls, addr: str) -> str:
        """
        Map NMEA address field (no leading $/!) to logical sentence type.
        Standard sentences use TTSSS (2-char talker + 3-char formatter), e.g. GNGGA -> GGA, GNVTG -> VTG.
        Proprietary (PAMTC) and non-standard lengths are returned unchanged.
        """
        if len(addr) == 5 and addr.isalpha() and addr.isupper():
            suffix = addr[2:]
            if suffix in cls._NMEA_THREE_LETTER_SENTENCES:
                return suffix
        return addr

    def __init__(self):
        self.serial_connection = None
        # Create two separate loggers
        self.nmea_logger = logging.getLogger('nmea')
        self.app_logger = logging.getLogger('app')
        
        self.nmea_messages = set()
        self.log_path = None
        self.state_path = None
        self.udp_socket = None
        self.is_streaming = False
        # Per-route counters. `streamed_messages` remains as a legacy total
        # (wind + gps) so existing callers keep working.
        self.streamed_messages = 0
        self.streamed_wind_messages = 0
        self.streamed_gps_messages = 0
        self.messages_received = 0  # Total NMEA messages received since connection
        self.connected_since = None  # epoch seconds when connection established
        self.message_history = []  # Store recent message history
        self.max_history = 100  # Maximum number of messages to keep in history
        # Last-seen timestamps for configured sentence IDs (used for UI sync)
        # { sentence_id: epoch_seconds_last_seen }
        self.sentence_last_seen = {}
        # Serial health/debug stats (helps diagnose "returned no data" bursts)
        self._serial_health_lock = threading.Lock()
        self.serial_health = {
            'last_good_nmea_ts': None,
            'last_read_attempt_ts': None,
            'last_raw_len': 0,
            'last_in_waiting': None,
            'read_timeouts': 0,
            'empty_reads': 0,
            'nodata_exceptions': 0,
            'other_read_exceptions': 0,
            'checksum_mismatch': 0,
            'checksum_missing': 0,
            'unmapped_messages': 0,
            'last_unmapped_type': None,
        }

        # SSE (Server-Sent Events) clients for near real-time UI updates
        self._sse_clients_lock = threading.Lock()
        self._sse_clients = set()  # set[queue.Queue]
        self._last_sensor_emit_ts = 0.0
        self._last_status_emit_ts = 0.0
        
        # Connection status tracking
        self.connection_status = self.CONN_STATUS_DISCONNECTED
        self.connection_message = ''
        self.connection_step_since = None  # epoch when current status phase began
        self.detected_baud = None
        self._cancel_connect = False  # flag checked by connect loops

        # Device identification (populated from $PAMTC,QPS / $PAMTC,QV after connect).
        # model_code is up to 10 ASCII chars per the WX manual; examples include
        # "300WX", "300WXIP", "300WXH", "200WX", "200WX-IPX7", "150WX", "100WX".
        # This extension was originally built for the 200WX; the 300WX baud switch
        # to 115200 is not supported on the 200WX (max 38400), so we surface a UI
        # notice when an older WX model is detected.
        self.device_info = None
        
        self.state = {
            'port': None,
            'baud_rate': 4800,
            'stay_at_4800': False,  # persisted; used by startup auto-connect
            'is_streaming': False,
            'sentence_config': {},  # { sentence_id: { "enabled": bool, "interval": int (tenths) } }
            # Persisted ArduRover autopilot setup (see mavlink_params.py).
            # Populated after the user picks SERIAL X/Y and applies once.
            # Shape: {
            #   'wind_serial': int|None,       # SERIALx for the wind (27001) UDPIN
            #   'gps_serial':  int|None,       # SERIALx for the GPS  (27002) UDPIN
            #   'use_gps_yaw_fallback': bool,  # also set EK3_SRC1_YAW=3
            #   'applied': bool,               # first apply succeeded at least once
            #   'expected': { param: value },  # snapshot to compare current vs
            #   'ignore_drift': bool,          # user chose Ignore -> never nag
            #   'last_apply_ts': float,
            #   'last_apply_result': {...},    # per-param apply outcome
            # }
            'autopilot_setup': {}
        }
        # Lock so only one consumer reads from serial (reader thread vs sentence query)
        self._serial_lock = threading.Lock()
        # Throttle "no data" serial read errors (log at most once per 30s)
        self._last_serial_read_error_log = 0

        # Cockpit data-lake WebSocket server (port 8765)
        self._ws_clients = set()
        self._ws_loop = None
        if HAS_WEBSOCKETS:
            self._start_ws_server()

        # mavlink2rest NAMED_VALUE_FLOAT publisher. This is what gets the
        # apparent / true wind into the autopilot's .BIN log as `NVF` rows
        # (WX_AppDir, WX_AppSpd, WX_TruDir, WX_TruSpd) -- independent of the
        # UDP NMEA stream so the log keeps recording even when the user has
        # the UDP stream toggled off, and named distinctly from ArduPilot's
        # own AppWndSpd / AppWndDir wind-vane emissions so the two streams
        # can be cross-checked post-flight. See app/mavlink_sender.py for
        # the collision-avoidance scheme (per-metric component_id).
        self._mav_sender = MavlinkSender() if HAS_MAVLINK_SENDER else None
        self._mav_publish_thread = None
        self._mav_publish_interval_s = 1.0
        # Mostly for diagnostics: number of NVFs accepted by mavlink2rest
        # since the publisher started, and the timestamp of the last accept.
        self._mav_publish_count = 0
        self._mav_publish_last_ok_ts = None

        # ArduRover parameter client. Only enabled if mavlink_params imported.
        # The user picks SERIAL X (wind) and SERIAL Y (GPS); the extension
        # writes matching ArduPilot parameters once and later checks for drift.
        self._param_client = ParamClient() if HAS_MAVLINK_PARAMS else None
        # Serialize check/apply/restore so overlapping UI clicks don't clobber
        # each other's state.
        self._autopilot_setup_lock = threading.Lock()
        
        # Aggregated sensor data for dashboard display
        self.sensor_data = {
            'wind_apparent': {
                'speed_kts': None,
                'angle': None,
                'source': None,
                'timestamp': None
            },
            'wind_true': {
                'speed_kts': None,
                'angle': None,  # Relative to vessel
                'direction_true': None,  # Relative to north
                'direction_magnetic': None,
                'source': None,
                'timestamp': None
            },
            'atmosphere': {
                'temperature_c': None,
                'humidity_pct': None,
                'pressure_bar': None,
                'dew_point_c': None,
                'source': None,
                'timestamp': None
            },
            'attitude': {
                'heading_true': None,
                'heading_magnetic': None,
                'pitch_deg': None,
                'roll_deg': None,
                'rate_of_turn': None,
                'source': None,
                'timestamp': None
            },
            'gps': {
                'latitude': None,
                'longitude': None,
                'altitude_m': None,
                'satellites': None,
                'fix_quality': None,
                'speed_kts': None,
                'course_true': None,
                'source': None,
                'timestamp': None
            },
            'time': {
                'utc_time': None,
                'utc_date': None,
                'source': None,
                'timestamp': None
            }
        }
        
        # Historical sensor data for sparklines (15 minutes = 900 seconds).
        # Samples are capped at ~1 Hz per series so a full payload stays small
        # even when NMEA arrives at 5–10 Hz.
        self.history_duration = 900  # seconds
        self.history_min_interval = 1.0  # seconds between samples per key
        self._history_last_record = {}  # key -> monotonic/time of last append
        self.sensor_history = {
            'wind_apparent_speed': [],
            'wind_apparent_angle': [],
            'wind_apparent_paired': [],  # {t, speed, angle} for rose/heatmap (MWV-R, bow-relative)
            'wind_true_paired': [],  # {t, speed, angle} MWV-T and/or MWD (true / north-ref)
            'wind_true_speed': [],
            'wind_true_direction': [],
            'temperature': [],
            'humidity': [],
            'pressure': [],
            'heading': [],
            'pitch': [],
            'roll': [],
            'rate_of_turn': [],
            'gps_speed': [],
            'gps_course': [],
            'satellites': []
        }
        
        # Thread control
        self.reader_thread = None
        self.should_stop = False
        
        # Configure logging. In production BlueOS binds /app/logs to a host
        # path that survives restarts. Tests override the location via
        # AIRMAR_WX_LOG_DIR so nothing needs write access to /app/.
        log_dir = Path(os.environ.get('AIRMAR_WX_LOG_DIR', '/app/logs'))
        log_dir.mkdir(parents=True, exist_ok=True)
        
        # Set up file handler for NMEA messages. Rotating on size so the
        # log cannot grow unbounded: 10 Hz per GPS sentence x 4 sentences
        # x ~80 chars = ~3.2 kB/s = ~275 MB/day. Rotating at 25 MB with
        # 4 backups caps disk use at ~125 MB and keeps ~10 hours of
        # history at the fast rate.
        self.log_path = log_dir / 'nmea_messages.log'
        nmea_fh = logging.handlers.RotatingFileHandler(
            self.log_path, mode='a',
            maxBytes=25 * 1024 * 1024, backupCount=4,
        )
        nmea_fh.setLevel(logging.INFO)
        nmea_formatter = logging.Formatter('%(asctime)s - %(message)s')
        nmea_fh.setFormatter(nmea_formatter)
        # Guard against duplicate handlers when NMEAHandler is
        # instantiated more than once (test suites, reload). Otherwise
        # each construction adds another FileHandler and every sentence
        # gets logged N times.
        for h in list(self.nmea_logger.handlers):
            self.nmea_logger.removeHandler(h)
            try:
                h.close()
            except Exception:
                pass
        self.nmea_logger.addHandler(nmea_fh)
        self.nmea_logger.setLevel(logging.INFO)
        # Per-line NMEA must not propagate to the root logger (avoids flooding
        # BlueOS extension / container logs at INFO for every sentence).
        self.nmea_logger.propagate = False
        
        # Set up file handler for application logs. Also rotating so a
        # noisy warning loop can't fill the disk between restarts.
        app_log_path = log_dir / '300wx.log'
        app_fh = logging.handlers.RotatingFileHandler(
            app_log_path, mode='a',
            maxBytes=5 * 1024 * 1024, backupCount=3,
        )
        app_fh.setLevel(logging.INFO)
        app_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        app_fh.setFormatter(app_formatter)
        for h in list(self.app_logger.handlers):
            self.app_logger.removeHandler(h)
            try:
                h.close()
            except Exception:
                pass
        self.app_logger.addHandler(app_fh)
        self.app_logger.setLevel(logging.INFO)

        # State file path
        self.state_path = log_dir / 'state.json'
        
        # Load saved state
        self.load_state()
        
        # Auto-connect in background so the web UI is always reachable
        t = threading.Thread(target=self._auto_connect, daemon=True, name='serial-auto')
        t.start()

        # 1 Hz NAMED_VALUE_FLOAT publisher (wind values into the autopilot
        # .BIN log via mavlink2rest). Independent of the UDP NMEA stream.
        if self._mav_sender is not None:
            self._mav_publish_thread = threading.Thread(
                target=self._mav_publish_loop,
                daemon=True,
                name='mav-nvf-publisher',
            )
            self._mav_publish_thread.start()
            try:
                planned = self._mav_sender.planned_component_ids()
                self.app_logger.info(
                    "mavlink2rest NVF publisher started at %.1f Hz; "
                    "system_id=255, component_ids=%s",
                    1.0 / self._mav_publish_interval_s, planned,
                )
            except Exception:
                pass

    def load_state(self):
        """Load saved state from file"""
        try:
            if self.state_path.exists():
                with open(self.state_path, 'r') as f:
                    loaded = json.load(f)
                self.state.update(loaded)
                # Older installs saved 38400; 300WX manual supports 115200 as max practical NMEA rate here.
                if self.state.get('baud_rate') == 38400:
                    self.state['baud_rate'] = self.OPERATING_BAUD_RATE
                    self.save_state()
                if 'stay_at_4800' not in self.state:
                    self.state['stay_at_4800'] = False
                if 'sentence_config' not in self.state or not isinstance(self.state['sentence_config'], dict):
                    self.state['sentence_config'] = {}
                # Migration: the extension used to pick one of two mutually
                # exclusive UDP modes ("windvane" / "gps") and shipped a
                # single stream to port 27000. Both routes are now always
                # active on 27001 (wind) and 27002 (GPS). Drop the obsolete
                # key silently so nothing downstream trips on it.
                if 'autopilot_mode' in self.state:
                    del self.state['autopilot_mode']
                if not isinstance(self.state.get('autopilot_setup'), dict):
                    self.state['autopilot_setup'] = {}
                # Migration (1.1.6): earlier versions wrote `GPS1_TYPE=5`
                # (NMEA), which forced the primary GPS driver into NMEA and
                # disabled the BlueBoat's onboard u-Blox. The new contract
                # is `GPS1_TYPE=1` (AUTO) + `GPS2_TYPE=5` (NMEA) so the
                # Airmar becomes GPS2 and the stock GPS keeps working.
                #
                # We do NOT silently write parameters at startup — that
                # would violate the apply-once model. Instead we invalidate
                # the persisted `expected` snapshot and clear the sticky
                # `ignore_drift` flag; the next check will then surface a
                # drift against the correct expectation, and the operator
                # can hit Restore to apply the new contract, or Ignore.
                self._migrate_autopilot_setup_gps_contract()
                self.is_streaming = self.state.get('is_streaming', False)
                self.app_logger.debug(f"Loaded state: port={self.state['port']}, baud={self.state['baud_rate']}, streaming={self.is_streaming}")
        except Exception as e:
            self.app_logger.error(f"Error loading state: {e}")

    def save_state(self):
        """Save current state to file"""
        try:
            self.state['is_streaming'] = self.is_streaming
            if 'sentence_config' not in self.state:
                self.state['sentence_config'] = {}
            with open(self.state_path, 'w') as f:
                json.dump(self.state, f)
            self.app_logger.debug(f"Saved state: port={self.state['port']}, baud={self.state['baud_rate']}")
        except Exception as e:
            self.app_logger.error(f"Error saving state: {e}")

    def _migrate_autopilot_setup_gps_contract(self):
        """Upgrade pre-1.1.6 autopilot_setup snapshots to the two-GPS contract.

        Old contract wrote `GPS1_TYPE=5` (NMEA) — that forced the primary
        GPS driver into NMEA and disabled the vehicle's onboard u-Blox.
        New contract: `GPS1_TYPE=1` (AUTO) so the stock u-Blox keeps
        working, and `GPS2_TYPE=5` (NMEA) for the Airmar.

        We deliberately do NOT re-POST parameters here — apply-once is a
        user-triggered action. Instead we:
          1. Drop the stale `expected` map and `last_apply_result` so the
             next drift check compares against the new contract, not the
             old one.
          2. Clear `ignore_drift` so the operator is notified once and can
             re-click Apply (or Ignore) with the corrected expectations.
          3. Keep `wind_serial`, `gps_serial`, and `use_gps_yaw_fallback`
             so the UI form comes up pre-populated with the previous
             selection.
        """
        setup = self.state.get('autopilot_setup')
        if not isinstance(setup, dict):
            return
        expected = setup.get('expected')
        if not isinstance(expected, dict):
            return
        # Two signals of the legacy contract: GPS1_TYPE explicitly set to
        # NMEA (5), OR GPS2_TYPE simply missing while a full apply was
        # persisted. Either one means "regenerate on next apply".
        legacy_gps1_nmea = expected.get('GPS1_TYPE') == 5.0
        missing_gps2 = 'GPS2_TYPE' not in expected
        if not (legacy_gps1_nmea or missing_gps2):
            return
        self.app_logger.info(
            "Migrating autopilot_setup to two-GPS contract "
            "(GPS1_TYPE=1 AUTO, GPS2_TYPE=5 NMEA). "
            "Previous expected snapshot dropped; next check will show "
            "drift so operator can re-Apply."
        )
        setup['expected'] = {}
        setup['last_apply_result'] = {}
        setup['last_apply_ts'] = None
        setup['applied'] = False
        setup['ignore_drift'] = False
        # Save immediately so a crash between here and the next save
        # doesn't leave a half-migrated state.json.
        try:
            with open(self.state_path, 'w') as f:
                json.dump(self.state, f)
        except Exception as e:
            self.app_logger.warning(
                "Migration save failed (will retry on next save): %s", e,
            )

    def _set_conn_status(self, status, message=''):
        """Update connection status with timestamp."""
        self.connection_status = status
        self.connection_message = message
        self.connection_step_since = time.time()

    def _auto_connect(self):
        """Background auto-connect: try saved port + baud hint first, then scan others.
        No UI click required; uses state.json from last successful session."""
        self._cancel_connect = False
        saved_port = self.state.get('port')
        saved_baud = self.state.get('baud_rate', 4800)
        stay_4800 = bool(self.state.get('stay_at_4800', False))

        # Try the saved port first (most likely to succeed)
        if saved_port and Path(saved_port).exists():
            self._set_conn_status(self.CONN_STATUS_AUTO_SCANNING,
                                  f'Trying saved port {saved_port}...')
            self.app_logger.info(
                f"Auto-connect: trying saved port {saved_port} "
                f"(baud hint {saved_baud}, stay_at_4800={stay_4800})")
            try:
                success, msg = self.connect_serial(
                    saved_port, saved_baud,
                    stay_at_4800=stay_4800,
                    max_attempts=6)
                if success:
                    self.app_logger.info(f"Auto-connect: restored {saved_port}")
                    return
            except Exception as e:
                self.app_logger.error(f"Auto-connect: saved port failed: {e}")

        # Saved port failed or missing — try every other available port once
        all_ports = self.get_ports()
        other_ports = [p for p in all_ports if p != saved_port]
        if not other_ports:
            if not saved_port or not Path(saved_port).exists():
                self._set_conn_status(self.CONN_STATUS_FAILED,
                                      'No serial ports found')
                self.app_logger.error("Auto-connect: no serial ports found")
            return

        self.app_logger.info(f"Auto-connect: scanning {len(other_ports)} other port(s)")
        for i, port in enumerate(other_ports, 1):
            if self._cancel_connect:
                self._set_conn_status(self.CONN_STATUS_DISCONNECTED, 'Cancelled')
                return
            self._set_conn_status(self.CONN_STATUS_AUTO_SCANNING,
                                  f'Scanning port {port} ({i}/{len(other_ports)})...')
            try:
                success, msg = self.connect_serial(
                    port, 4800,
                    stay_at_4800=stay_4800,
                    max_attempts=6)
                if success:
                    self.app_logger.info(f"Auto-connect: connected on {port}")
                    return
            except Exception as e:
                self.app_logger.debug(f"Auto-connect: {port} failed: {e}")

        self._set_conn_status(self.CONN_STATUS_FAILED,
                              'No device responded on any port')
        self.app_logger.error("Auto-connect: no device responded on any port")

    def start_reader_thread(self):
        """Start the background thread for reading serial data"""
        if self.reader_thread is None or not self.reader_thread.is_alive():
            self.should_stop = False
            self.reader_thread = threading.Thread(target=self._read_serial_loop)
            self.reader_thread.daemon = True
            self.reader_thread.start()
            self.app_logger.debug("Started serial reader thread")

    def stop_reader_thread(self):
        """Stop the background thread"""
        self.should_stop = True
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
            self.app_logger.debug("Stopped serial reader thread")

    def _split_nmea_sentences(self, data):
        """Split a read buffer into individual NMEA sentences (one per yield).
        Handles $... and !... (AIS etc.), multiple sentences per read, and CR/LF."""
        if not data or ('$' not in data and '!' not in data):
            return
        for line in re.split(r'[\r\n]+', data):
            line = line.strip()
            if not line:
                continue
            starts = [m.start() for m in re.finditer(r'[$!]', line)]
            for i, start in enumerate(starts):
                end = starts[i + 1] if i + 1 < len(starts) else len(line)
                frag = line[start:end].strip()
                if frag and ',' in frag:
                    yield frag

    def _map_msg_to_sentence_id(self, raw_line, msg_type):
        """Map an incoming NMEA msg to a device sentence_id (SUPPORTED_SENTENCES key).
        Returns None if no reliable mapping exists."""
        try:
            if not msg_type:
                return None

            # MWV / WIMWV maps to MWVR/MWVT depending on R/T field
            if msg_type in ('MWV', 'WIMWV'):
                fields = raw_line.split('*')[0].split(',')
                ref = fields[2] if len(fields) > 2 else ''
                if ref == 'T':
                    return 'MWVT'
                # Default to apparent/relative
                return 'MWVR'

            # Most sentences map by the 3-letter sentence code (talker prefix ignored)
            code = msg_type[-3:] if len(msg_type) >= 3 else None
            if code and code in self.SUPPORTED_SENTENCES:
                return code
            return None
        except Exception:
            return None

    def _nmea_checksum_ok(self, raw_line):
        """Return True if checksum matches, False if mismatch, None if missing/invalid (NMEA 0183: $ or !)."""
        try:
            if not raw_line or '*' not in raw_line:
                return None
            if not (raw_line.startswith('$') or raw_line.startswith('!')):
                return None
            body, chk = raw_line[1:].split('*', 1)
            chk = chk.strip()
            if len(chk) < 2:
                return None
            expected = int(chk[:2], 16)
            calc = 0
            for b in body.encode('ascii', errors='ignore'):
                calc ^= b
            return calc == expected
        except Exception:
            return None

    def _incoming_line_checksum_valid(self, line):
        """300WX manual: received sentences must include a matching checksum."""
        if not self._line_looks_like_nmea(line):
            return False
        return self._nmea_checksum_ok(line) is True

    def get_serial_health(self):
        """Return current serial health/debug stats."""
        with self._serial_health_lock:
            h = dict(self.serial_health)
        now = time.time()
        h['seconds_since_last_good_nmea'] = (now - h['last_good_nmea_ts']) if h.get('last_good_nmea_ts') else None
        h['connected_since'] = self.connected_since
        h['port'] = self.state.get('port')
        h['baud_rate'] = self.state.get('baud_rate')
        return h

    def _sse_broadcast(self, event_name, payload):
        """Send an event to all connected SSE clients."""
        if not event_name:
            return
        msg = {'event': event_name, 'data': payload}
        with self._sse_clients_lock:
            clients = list(self._sse_clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                # Drop oldest and try again
                try:
                    q.get_nowait()
                except Exception:
                    pass
                try:
                    q.put_nowait(msg)
                except Exception:
                    pass

    def sse_add_client(self, max_queue=500):
        q = queue.Queue(maxsize=max_queue)
        with self._sse_clients_lock:
            self._sse_clients.add(q)
        return q

    def sse_remove_client(self, q):
        with self._sse_clients_lock:
            try:
                self._sse_clients.discard(q)
            except Exception:
                pass

    def _emit_sensor_if_due(self):
        now = time.time()
        # At most 5 Hz to avoid UI thrash
        if now - self._last_sensor_emit_ts >= 0.2:
            self._last_sensor_emit_ts = now
            self._sse_broadcast('sensor_data', self.sensor_data)

    def _emit_status_if_due(self):
        now = time.time()
        # At most 1 Hz for status (counts/health)
        if now - self._last_status_emit_ts >= 1.0:
            self._last_status_emit_ts = now
            self._sse_broadcast('stream_status', self._stream_status_snapshot(now=now))

    def _stream_status_snapshot(self, now=None):
        """Single source of truth for stream status (SSE, REST, init)."""
        if now is None:
            now = time.time()
        return {
            'is_streaming': self.is_streaming,
            'routes': self.get_stream_routes(),
            'streamed_messages': self.streamed_messages,
            'streamed_wind_messages': self.streamed_wind_messages,
            'streamed_gps_messages': self.streamed_gps_messages,
            'messages_received': self.messages_received,
            'serial_health': self.get_serial_health(),
            'observed_sentence_last_seen': self.sentence_last_seen,
            'mavlink_nvf': self.get_mavlink_nvf_status(),
            'autopilot_setup': self.get_autopilot_setup_status(),
            'now': now,
            'connected_since': self.connected_since,
        }

    def _read_serial_loop(self):
        """Background thread function for reading serial data. Processes messages as fast as
        they arrive; only sleeps when read returns no data to avoid busy-loop on USB quirks."""
        while not self.should_stop:
            if self.serial_connection and self.serial_connection.is_open:
                got_data = False
                try:
                    with self._serial_health_lock:
                        self.serial_health['last_read_attempt_ts'] = time.time()
                    with self._serial_lock:
                        raw = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                        try:
                            with self._serial_health_lock:
                                self.serial_health['last_in_waiting'] = getattr(self.serial_connection, 'in_waiting', None)
                        except Exception:
                            pass
                    with self._serial_health_lock:
                        self.serial_health['last_raw_len'] = len(raw) if raw else 0
                        if not raw:
                            self.serial_health['read_timeouts'] += 1
                    for data in self._split_nmea_sentences(raw):
                        got_data = True
                        chk_ok = self._nmea_checksum_ok(data)
                        with self._serial_health_lock:
                            if chk_ok is True:
                                self.serial_health['last_good_nmea_ts'] = time.time()
                            elif chk_ok is False:
                                self.serial_health['checksum_mismatch'] += 1
                            else:
                                self.serial_health['checksum_missing'] += 1
                        # Per 300WX manual: only accept sentences with valid checksum
                        if chk_ok is not True:
                            continue

                        self.messages_received += 1
                        # Parse NMEA message type (letters only; avoids HCHDG31.0 from truncated lines)
                        raw_type = data.split(',')[0].lstrip('$!').strip()
                        m = re.match(r'^[A-Z]+', raw_type) if raw_type else None
                        addr = m.group(0) if m else (raw_type or '')
                        msg_type = self._nmea_sentence_formatter(addr)
                        self.nmea_messages.add(msg_type)
                        # Track sentence last-seen for UI auto-sync
                        sentence_id = self._map_msg_to_sentence_id(data, msg_type)
                        if sentence_id:
                            self.sentence_last_seen[sentence_id] = time.time()
                        else:
                            with self._serial_health_lock:
                                self.serial_health['unmapped_messages'] += 1
                                self.serial_health['last_unmapped_type'] = msg_type
                        # Update aggregated sensor data
                        self._parse_nmea_for_dashboard(data, msg_type)
                        
                        # Add message to history
                        message = {
                            "raw": data,
                            "type": msg_type,
                            "timestamp": datetime.datetime.now().isoformat()
                        }
                        self.message_history.insert(0, message)  # Add to start of list
                        if len(self.message_history) > self.max_history:
                            self.message_history.pop()  # Remove oldest message
                        
                        # Log the message
                        self.log_message(data)
                        # Push to UI (near real-time)
                        self._sse_broadcast('nmea_message', message)
                        # Push derived aggregates/status (throttled)
                        self._emit_sensor_if_due()
                        self._emit_status_if_due()
                        # Forward to autopilot via UDP. Both routes are
                        # always active: `stream_message` picks the right
                        # port (27001 for wind, 27002 for GPS) or skips
                        # sentences that neither driver cares about.
                        if self.is_streaming:
                            self.stream_message(data, msg_type)
                except Exception as e:
                    err_str = str(e)
                    # Throttle "read but no data" to avoid log flood and possible I/O contention
                    if "returned no data" in err_str or "multiple access" in err_str:
                        with self._serial_health_lock:
                            self.serial_health['nodata_exceptions'] += 1
                        now = time.time()
                        if now - self._last_serial_read_error_log >= 30:
                            self._last_serial_read_error_log = now
                            health = self.get_serial_health()
                            self.app_logger.warning(
                                "Serial read: %s (throttled). port=%s baud=%s in_waiting=%s "
                                "since_last_good_nmea=%s timeouts=%s empty=%s nodata_ex=%s cksum_mismatch=%s",
                                err_str,
                                health.get('port'),
                                health.get('baud_rate'),
                                health.get('last_in_waiting'),
                                health.get('seconds_since_last_good_nmea'),
                                health.get('read_timeouts'),
                                health.get('empty_reads'),
                                health.get('nodata_exceptions'),
                                health.get('checksum_mismatch'),
                            )
                        time.sleep(0.02)  # Brief sleep only on spurious read so we don't tight-loop
                    else:
                        with self._serial_health_lock:
                            self.serial_health['other_read_exceptions'] += 1
                        self.app_logger.error(f"Error in serial reader thread: {e}")
                if not got_data and not self.should_stop:
                    # No message this iteration (timeout or empty line); short sleep to avoid busy-wait
                    with self._serial_health_lock:
                        self.serial_health['empty_reads'] += 1
                    time.sleep(0.01)
            else:
                time.sleep(0.2)  # Not connected; sleep before rechecking

    def _parse_nmea_for_dashboard(self, raw_data, msg_type):
        """
        Parse NMEA message and update aggregated sensor data for dashboard.
        """
        try:
            # Remove checksum if present
            data = raw_data.split('*')[0]
            fields = data.split(',')
            timestamp = datetime.datetime.now().isoformat()
            
            # MWV / WIMWV - Wind Speed and Angle (Relative or True)
            if msg_type in ('MWV', 'WIMWV'):
                # $WIMWV,angle,R/T,speed,unit,status*CC
                if len(fields) >= 6 and fields[5] == 'A':  # Valid data
                    angle = float(fields[1]) if fields[1] else None
                    speed = float(fields[3]) if fields[3] else None
                    reference = fields[2]  # R=Relative, T=True/Theoretical
                    
                    if reference == 'R':
                        self.sensor_data['wind_apparent']['angle'] = angle
                        self.sensor_data['wind_apparent']['speed_kts'] = speed
                        self.sensor_data['wind_apparent']['source'] = 'WIMWV'
                        self.sensor_data['wind_apparent']['timestamp'] = timestamp
                        self._record_history('wind_apparent_speed', speed)
                        self._record_history('wind_apparent_angle', angle)
                        self._record_wind_paired_samples('wind_apparent_paired', speed, angle)
                    elif reference == 'T':
                        self.sensor_data['wind_true']['angle'] = angle
                        self.sensor_data['wind_true']['speed_kts'] = speed
                        self.sensor_data['wind_true']['source'] = 'WIMWV'
                        self.sensor_data['wind_true']['timestamp'] = timestamp
                        self._record_history('wind_true_speed', speed)
                        # The MWV-T angle is BOW-RELATIVE, but the true wind
                        # rose is north-referenced (same frame as $WIMWD).
                        # Convert to a true-north "wind from" bearing using the
                        # current true heading so both sources agree. Without a
                        # heading we cannot north-reference it, so we skip the
                        # rose sample to avoid plotting a bow-relative angle in
                        # a north-referenced chart (which produced a phantom
                        # second wedge ~180 deg from the real wind).
                        heading_true = self.sensor_data['attitude'].get('heading_true')
                        if angle is not None and heading_true is not None:
                            dir_from_true = (angle + heading_true) % 360
                            self._record_wind_paired_samples(
                                'wind_true_paired', speed, dir_from_true
                            )
            
            # MWD / WIMWD - Wind Direction and Speed (True, relative to north)
            elif msg_type in ('MWD', 'WIMWD'):
                # $WIMWD,dir_true,T,dir_mag,M,speed_kts,N,speed_ms,M*CC
                if len(fields) >= 8:
                    dir_true = float(fields[1]) if fields[1] else None
                    dir_mag = float(fields[3]) if fields[3] else None
                    speed_kts = float(fields[5]) if fields[5] else None
                    
                    self.sensor_data['wind_true']['direction_true'] = dir_true
                    self.sensor_data['wind_true']['direction_magnetic'] = dir_mag
                    if speed_kts is not None:
                        self.sensor_data['wind_true']['speed_kts'] = speed_kts
                    self.sensor_data['wind_true']['source'] = 'WIMWD'
                    self.sensor_data['wind_true']['timestamp'] = timestamp
                    self._record_history('wind_true_direction', dir_true)
                    self._record_history('wind_true_speed', speed_kts)
                    if dir_true is not None and speed_kts is not None:
                        self._record_wind_paired_samples(
                            'wind_true_paired', speed_kts, dir_true
                        )
                    # Broadcast to Cockpit data-lake
                    if dir_true is not None:
                        self.ws_broadcast('wind-direction-true', round(dir_true, 1))
                    if speed_kts is not None:
                        self.ws_broadcast('wind-speed-kts', round(speed_kts, 1))
            
            # MDA / WIMDA - Meteorological Composite
            elif msg_type in ('MDA', 'WIMDA'):
                # $WIMDA,baro_in,I,baro_bar,B,air_temp,C,water_temp,C,humidity,%,dew_point,C,...
                if len(fields) >= 12:
                    pressure_bar = float(fields[3]) if fields[3] else None
                    temp_c = float(fields[5]) if fields[5] else None
                    humidity = float(fields[9]) if fields[9] else None
                    dew_point = float(fields[11]) if fields[11] else None
                    
                    self.sensor_data['atmosphere']['pressure_bar'] = pressure_bar
                    self.sensor_data['atmosphere']['temperature_c'] = temp_c
                    self.sensor_data['atmosphere']['humidity_pct'] = humidity
                    self.sensor_data['atmosphere']['dew_point_c'] = dew_point
                    self.sensor_data['atmosphere']['source'] = 'WIMDA'
                    self.sensor_data['atmosphere']['timestamp'] = timestamp
                    self._record_history('temperature', temp_c)
                    self._record_history('humidity', humidity)
                    self._record_history('pressure', pressure_bar)
            
            # HDT / HCHDT / GNHDT - Heading True
            elif msg_type in ('HDT', 'HCHDT'):
                # $HCHDT,heading,T*CC
                if len(fields) >= 2 and fields[1]:
                    heading = float(fields[1])
                    self.sensor_data['attitude']['heading_true'] = heading
                    self.sensor_data['attitude']['source'] = 'HCHDT'
                    self.sensor_data['attitude']['timestamp'] = timestamp
                    self._record_history('heading', heading)
                    self.ws_broadcast('heading-true', round(heading, 1))
            
            # HDG / HCHDG / CHDG - Heading Magnetic
            elif msg_type in ('HDG', 'HCHDG', 'CHDG'):
                # $HCHDG,heading,dev,E/W,var,E/W*CC
                if len(fields) >= 2 and fields[1]:
                    heading = float(fields[1])
                    self.sensor_data['attitude']['heading_magnetic'] = heading
                    if self.sensor_data['attitude']['source'] != 'HCHDT':
                        self.sensor_data['attitude']['source'] = msg_type
                        self.sensor_data['attitude']['timestamp'] = timestamp
            
            # XDR / YXXDR - Transducer Measurements (Pitch/Roll from type B)
            elif msg_type in ('XDR', 'YXXDR'):
                # Parse in groups of 4: type, value, unit, name
                i = 1
                while i + 3 < len(fields):
                    name = fields[i + 3] if i + 3 < len(fields) else ''
                    value = fields[i + 1] if i + 1 < len(fields) and fields[i + 1] else None
                    
                    if value is not None:
                        value = float(value)
                        if name == 'PTCH':
                            self.sensor_data['attitude']['pitch_deg'] = value
                            self.sensor_data['attitude']['source'] = 'YXXDR'
                            self.sensor_data['attitude']['timestamp'] = timestamp
                            self._record_history('pitch', value)
                        elif name == 'ROLL':
                            self.sensor_data['attitude']['roll_deg'] = value
                            self.sensor_data['attitude']['source'] = 'YXXDR'
                            self.sensor_data['attitude']['timestamp'] = timestamp
                            self._record_history('roll', value)
                    i += 4
            
            # ROT / TIROT - Rate of Turn
            elif msg_type in ('ROT', 'TIROT'):
                # $TIROT,rate,status*CC
                if len(fields) >= 3 and fields[2] == 'A' and fields[1]:
                    rate = float(fields[1])
                    self.sensor_data['attitude']['rate_of_turn'] = rate
                    self._record_history('rate_of_turn', rate)
                    if 'source' not in self.sensor_data['attitude'] or self.sensor_data['attitude']['source'] not in ['HCHDT', 'YXXDR']:
                        self.sensor_data['attitude']['source'] = 'TIROT'
                        self.sensor_data['attitude']['timestamp'] = timestamp
            
            # GGA / GPGGA / GNGGA - GPS Fix Data
            elif msg_type in ('GGA', 'GPGGA', 'GNGGA'):
                # $GPGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,...
                if len(fields) >= 10:
                    lat = self._parse_nmea_coord(fields[2], fields[3]) if fields[2] else None
                    lon = self._parse_nmea_coord(fields[4], fields[5]) if fields[4] else None
                    fix_quality = int(fields[6]) if fields[6] else 0
                    satellites = int(fields[7]) if fields[7] else None
                    altitude = float(fields[9]) if fields[9] else None
                    
                    quality_names = {0: 'Invalid', 1: 'GPS Fix', 2: 'DGPS', 4: 'RTK Fixed', 5: 'RTK Float'}
                    
                    self.sensor_data['gps']['latitude'] = lat
                    self.sensor_data['gps']['longitude'] = lon
                    self.sensor_data['gps']['fix_quality'] = quality_names.get(fix_quality, str(fix_quality))
                    self.sensor_data['gps']['satellites'] = satellites
                    self.sensor_data['gps']['altitude_m'] = altitude
                    self.sensor_data['gps']['source'] = 'GGA'
                    self.sensor_data['gps']['timestamp'] = timestamp
                    self._record_history('satellites', satellites)
                    if lat is not None:
                        self.ws_broadcast('gps-latitude', round(lat, 7))
                    if lon is not None:
                        self.ws_broadcast('gps-longitude', round(lon, 7))
                    if altitude is not None:
                        self.ws_broadcast('gps-altitude-m', round(altitude, 1))
                    self.ws_broadcast('gps-satellites', satellites)
                    self.ws_broadcast('gps-fix-quality', fix_quality)
            
            # VTG / GPVTG / GNVTG - Course Over Ground and Ground Speed
            elif msg_type in ('VTG', 'GPVTG', 'GNVTG'):
                # $GPVTG,track_true,T,track_mag,M,speed_kts,N,speed_kmh,K,mode*CC
                if len(fields) >= 6:
                    course = float(fields[1]) if fields[1] else None
                    speed_kts = float(fields[5]) if fields[5] else None
                    
                    self.sensor_data['gps']['course_true'] = course
                    self.sensor_data['gps']['speed_kts'] = speed_kts
                    self._record_history('gps_speed', speed_kts)
                    self._record_history('gps_course', course)
                    if course is not None:
                        self.ws_broadcast('gps-course-true', round(course, 1))
                    if speed_kts is not None:
                        self.ws_broadcast('gps-speed-kts', round(speed_kts, 1))
                    if self.sensor_data['gps']['source'] != 'GGA':
                        self.sensor_data['gps']['source'] = 'VTG'
                        self.sensor_data['gps']['timestamp'] = timestamp
            
            # ZDA / GPZDA / GNZDA - Time and Date
            elif msg_type in ('ZDA', 'GPZDA', 'GNZDA'):
                # $--ZDA,hhmmss,dd,mm,yyyy,tz_h,tz_m*CC
                if len(fields) >= 5:
                    time_str = fields[1]
                    day = fields[2]
                    month = fields[3]
                    year = fields[4]
                    
                    if time_str and len(time_str) >= 6:
                        utc_time = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:6]}"
                        self.sensor_data['time']['utc_time'] = utc_time
                    
                    if day and month and year:
                        utc_date = f"{year}-{month.zfill(2)}-{day.zfill(2)}"
                        self.sensor_data['time']['utc_date'] = utc_date
                    
                    self.sensor_data['time']['source'] = 'ZDA'
                    self.sensor_data['time']['timestamp'] = timestamp
                    
        except Exception as e:
            # Silently ignore parse errors to not spam logs
            pass

    def _record_history(self, key, value):
        """Record a value to the history buffer for sparklines (~1 Hz max)."""
        if value is None:
            return

        now = time.time()
        last = self._history_last_record.get(key)
        if last is not None and (now - last) < self.history_min_interval:
            return
        self._history_last_record[key] = now
        self.sensor_history[key].append({'t': now, 'v': value})

        # Prune old entries (older than 15 minutes)
        cutoff = now - self.history_duration
        self.sensor_history[key] = [
            entry for entry in self.sensor_history[key]
            if entry['t'] >= cutoff
        ]

    def _record_wind_paired_samples(self, buffer_key, speed, angle):
        """Append one timestamped sample for wind rose / heatmap (~1 Hz max)."""
        if speed is None or angle is None:
            return
        now = time.time()
        last = self._history_last_record.get(buffer_key)
        if last is not None and (now - last) < self.history_min_interval:
            return
        self._history_last_record[buffer_key] = now
        self.sensor_history[buffer_key].append({
            't': now,
            'speed': float(speed),
            'angle': float(angle),
        })
        cutoff = now - self.history_duration
        self.sensor_history[buffer_key] = [
            e for e in self.sensor_history[buffer_key]
            if e['t'] >= cutoff
        ]

    def get_sensor_history(self, keys=None):
        """Return sensor history for sparklines.

        If ``keys`` is a non-empty iterable of series names, only those series
        are included (plus ``history_window_seconds``). Unknown keys are ignored.
        """
        now = time.time()
        if keys:
            selected = [k for k in keys if k in self.sensor_history]
        else:
            selected = list(self.sensor_history.keys())

        result = {}
        for key in selected:
            entries = self.sensor_history[key]
            if key in ('wind_apparent_paired', 'wind_true_paired'):
                result[key] = [
                    {
                        't': round(now - e['t']),
                        'speed': e['speed'],
                        'angle': e['angle'],
                    }
                    for e in entries
                ]
                continue
            result[key] = [
                {'t': round(now - entry['t']), 'v': entry['v']}
                for entry in entries
            ]
        result['history_window_seconds'] = int(self.history_duration)
        return result

    def reset_wind_paired_history(self, which):
        """Clear paired samples used for wind roses / heatmaps (``apparent`` or ``true``)."""
        if which == 'apparent':
            self.sensor_history['wind_apparent_paired'] = []
            self._history_last_record.pop('wind_apparent_paired', None)
        elif which == 'true':
            self.sensor_history['wind_true_paired'] = []
            self._history_last_record.pop('wind_true_paired', None)

    def _parse_nmea_coord(self, coord_str, direction):
        """Convert NMEA coordinate (DDMM.MMMM) to decimal degrees."""
        if not coord_str:
            return None
        try:
            # Find decimal point position
            dot_pos = coord_str.index('.')
            degrees = int(coord_str[:dot_pos - 2])
            minutes = float(coord_str[dot_pos - 2:])
            decimal = degrees + (minutes / 60)
            
            if direction in ['S', 'W']:
                decimal = -decimal
            
            return round(decimal, 6)
        except:
            return None

    def get_sensor_data(self):
        """Return the current aggregated sensor data for dashboard."""
        return self.sensor_data

    def start_streaming(self):
        """Start dual-route UDP streaming (idempotent).

        Wind sentences go to `UDP_HOST:UDP_WIND_PORT` (27001) and GPS/
        heading sentences go to `UDP_HOST:UDP_GPS_PORT` (27002). Both
        routes share a single socket; each has its own counter.
        """
        try:
            if not self.udp_socket:
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if not self.is_streaming:
                # Reset counters only when actually starting so a re-toggle
                # after a stop starts fresh, but redundant `start` calls do
                # not zero the accumulators mid-run.
                self.streamed_messages = 0
                self.streamed_wind_messages = 0
                self.streamed_gps_messages = 0
            self.is_streaming = True
            self.state['is_streaming'] = True
            self.save_state()
            self.app_logger.info(
                "UDP streaming started — wind %s → %s:%d, gps %s → %s:%d",
                sorted(self.UDP_WIND_SENTENCES), self.UDP_HOST, self.UDP_WIND_PORT,
                sorted(self.UDP_GPS_SENTENCES), self.UDP_HOST, self.UDP_GPS_PORT,
            )
            return True, "Streaming started"
        except Exception as e:
            self.app_logger.error(f"Error starting UDP stream: {e}")
            return False, str(e)

    def stop_streaming(self):
        """Stop UDP streaming"""
        try:
            if self.udp_socket:
                self.udp_socket.close()
                self.udp_socket = None
            self.is_streaming = False
            self.state['is_streaming'] = False
            self.save_state()
            self.app_logger.info("UDP streaming stopped")
            return True, "Streaming stopped"
        except Exception as e:
            self.app_logger.error(f"Error stopping UDP stream: {e}")
            return False, str(e)

    def _route_for_sentence(self, msg_type):
        """Return (port, counter_attr) if `msg_type` maps to a UDP route, else None.

        `msg_type` is the normalized sentence tail from `_nmea_sentence_formatter`
        (e.g. 'MWV', 'GGA'). Talker-prefixed forms like 'WIMWV' are still
        matched because we compare the last three characters, which is what
        the previous single-mode implementation did.
        """
        if len(msg_type) < 3:
            return None
        tail = msg_type[-3:]
        if tail in self.UDP_WIND_SENTENCES:
            return (self.UDP_WIND_PORT, 'streamed_wind_messages')
        if tail in self.UDP_GPS_SENTENCES:
            return (self.UDP_GPS_PORT, 'streamed_gps_messages')
        return None

    def get_stream_routes(self):
        """UI/status snapshot of both UDP routes and their counters."""
        return [
            {
                'name': 'wind',
                'sentences': sorted(self.UDP_WIND_SENTENCES),
                'host': self.UDP_HOST,
                'port': self.UDP_WIND_PORT,
                'endpoint': f'{self.UDP_HOST}:{self.UDP_WIND_PORT}',
                'streamed_messages': self.streamed_wind_messages,
            },
            {
                'name': 'gps',
                'sentences': sorted(self.UDP_GPS_SENTENCES),
                'host': self.UDP_HOST,
                'port': self.UDP_GPS_PORT,
                'endpoint': f'{self.UDP_HOST}:{self.UDP_GPS_PORT}',
                'streamed_messages': self.streamed_gps_messages,
            },
        ]

    def get_autopilot_setup_status(self):
        """Snapshot of persisted ArduRover setup state for the UI.

        The heavy lifting (parameter check/apply, drift detection) lives in
        :mod:`mavlink_params`; this accessor just surfaces the persisted
        selection so status endpoints and SSE can render the current setup
        without touching mavlink2rest on every request.
        """
        setup = self.state.get('autopilot_setup') or {}
        return {
            'enabled': bool(self._param_client is not None),
            'wind_serial': setup.get('wind_serial'),
            'gps_serial': setup.get('gps_serial'),
            'use_gps_yaw_fallback': bool(setup.get('use_gps_yaw_fallback', False)),
            'applied': bool(setup.get('applied', False)),
            'ignore_drift': bool(setup.get('ignore_drift', False)),
            'expected': setup.get('expected') or {},
            'last_apply_ts': setup.get('last_apply_ts'),
            'last_apply_result': setup.get('last_apply_result') or {},
            'min_serial_index': _MIN_SERIAL_INDEX if HAS_MAVLINK_PARAMS else 1,
            'max_serial_index': _MAX_SERIAL_INDEX if HAS_MAVLINK_PARAMS else 9,
            'udp_wind_port': self.UDP_WIND_PORT,
            'udp_gps_port': self.UDP_GPS_PORT,
            'udpin_wind_string': f'udpin:0.0.0.0:{self.UDP_WIND_PORT}',
            'udpin_gps_string': f'udpin:0.0.0.0:{self.UDP_GPS_PORT}',
        }

    # ── ArduRover parameter setup ────────────────────────────────────
    #
    # Flow:
    #   1) UI POSTs SERIAL X / SERIAL Y (and optional yaw fallback flag).
    #   2) `apply_autopilot_setup` writes each expected param via mavlink2rest
    #      (verified by a fresh PARAM_VALUE echo). The subset of params the
    #      autopilot actually has is persisted as `expected`.
    #   3) `check_autopilot_setup` compares live values against `expected`
    #      and reports drift; the UI shows a Restore/Ignore banner.
    #   4) `restore_autopilot_setup` re-applies `expected`.
    #      `ignore_autopilot_drift` sets `ignore_drift=True` so the banner
    #      never shows for this X/Y pair again.
    #
    # Changing X, Y, or the yaw-fallback flag creates a new expected snapshot
    # and clears `ignore_drift` — that is a new setup, not the same one
    # drifting.

    def _apply_and_persist(self, wind_serial, gps_serial, use_gps_yaw_fallback):
        """Do the actual PARAM_SET pass and persist the expected snapshot.

        `applied` on the persisted setup tracks whether we successfully
        POSTed any writes to mavlink2rest — verified or not. It does NOT
        track whether the PARAM_VALUE echo transport is working, because
        that turned out to be unreliable across mavlink2rest builds and
        caused every apply to look like it did nothing.
        """
        expected = build_expected_params(
            int(wind_serial), int(gps_serial), bool(use_gps_yaw_fallback),
        )
        result = self._param_client.apply_expected(expected)
        # Snapshot covers every param whose POST succeeded, so the drift
        # check has something to compare against later even when the
        # PARAM_VALUE echo did not arrive in time during apply.
        snapshot = snapshot_from_apply_result(result)
        posted_any = any(row.get('ok') for row in result.values())
        setup = self.state.get('autopilot_setup') or {}
        setup.update({
            'wind_serial': int(wind_serial),
            'gps_serial': int(gps_serial),
            'use_gps_yaw_fallback': bool(use_gps_yaw_fallback),
            'applied': bool(posted_any),
            'ignore_drift': False,
            'expected': snapshot,
            'last_apply_ts': time.time(),
            'last_apply_result': result,
        })
        self.state['autopilot_setup'] = setup
        self.save_state()
        # Bucket outcomes for a concise log line.
        counts = {'noop': 0, 'wrote': 0, 'wrote_unverified': 0, 'failed': 0}
        for row in result.values():
            action = row.get('action') or 'failed'
            counts[action] = counts.get(action, 0) + 1
        self.app_logger.info(
            "ArduRover setup applied: wind=SERIAL%d gps=SERIAL%d yaw_fallback=%s "
            "counts=%s expected=%s",
            wind_serial, gps_serial, use_gps_yaw_fallback,
            counts, sorted(snapshot.keys()),
        )
        return result, snapshot

    def apply_autopilot_setup(self, wind_serial, gps_serial, use_gps_yaw_fallback):
        """User-triggered apply for a chosen SERIAL X/Y (+ optional yaw)."""
        if not HAS_MAVLINK_PARAMS or self._param_client is None:
            return False, 'mavlink parameter client not available', {}
        ok, err = validate_selection(wind_serial, gps_serial)
        if not ok:
            return False, err, {}
        with self._autopilot_setup_lock:
            try:
                result, snapshot = self._apply_and_persist(
                    wind_serial, gps_serial, use_gps_yaw_fallback,
                )
            except Exception as e:
                self.app_logger.error(f"apply_autopilot_setup failed: {e}")
                return False, str(e), {}
        return True, 'applied', {
            'result': result,
            'expected': snapshot,
            'status': self.get_autopilot_setup_status(),
        }

    def check_autopilot_setup(self):
        """Compare live autopilot params against the persisted expected set.

        If every read returned no PARAM_VALUE echo we treat that as a
        transport failure (mavlink2rest wedged, autopilot silent, etc.)
        rather than reporting every param as drifted — the latter would
        panic the user into hitting Restore repeatedly on nothing.
        """
        if not HAS_MAVLINK_PARAMS or self._param_client is None:
            return False, 'mavlink parameter client not available', {}
        setup = self.state.get('autopilot_setup') or {}
        expected = setup.get('expected') or {}
        if not expected:
            return True, 'no setup applied yet', {
                'expected': {},
                'current': {},
                'drift': [],
                'ignore_drift': bool(setup.get('ignore_drift', False)),
                'status': self.get_autopilot_setup_status(),
            }
        with self._autopilot_setup_lock:
            current = self._param_client.read_expected(expected)
        # Transport-failure short-circuit: if not one param echoed back,
        # we can't tell whether they drifted or the transport is broken.
        # Report the ambiguity honestly instead of manufacturing drift.
        if current and not any(row.get('available') for row in current.values()):
            return True, (
                'unable to read autopilot params via mavlink2rest '
                '(no PARAM_VALUE echoes received); check that mavlink2rest is '
                'running and the autopilot is connected'
            ), {
                'expected': expected,
                'current': current,
                'drift': [],
                'ignore_drift': bool(setup.get('ignore_drift', False)),
                'status': self.get_autopilot_setup_status(),
                'transport_ok': False,
            }
        drift = diff_current_vs_expected(current, expected)
        return True, 'ok', {
            'expected': expected,
            'current': current,
            'drift': drift,
            'ignore_drift': bool(setup.get('ignore_drift', False)),
            'status': self.get_autopilot_setup_status(),
            'transport_ok': True,
        }

    def restore_autopilot_setup(self):
        """Re-apply the persisted expected snapshot (Restore button)."""
        if not HAS_MAVLINK_PARAMS or self._param_client is None:
            return False, 'mavlink parameter client not available', {}
        setup = self.state.get('autopilot_setup') or {}
        wind_serial = setup.get('wind_serial')
        gps_serial = setup.get('gps_serial')
        use_gps_yaw_fallback = bool(setup.get('use_gps_yaw_fallback', False))
        ok, err = validate_selection(wind_serial, gps_serial)
        if not ok:
            return False, f'no valid setup to restore: {err}', {}
        with self._autopilot_setup_lock:
            try:
                result, snapshot = self._apply_and_persist(
                    wind_serial, gps_serial, use_gps_yaw_fallback,
                )
            except Exception as e:
                self.app_logger.error(f"restore_autopilot_setup failed: {e}")
                return False, str(e), {}
        return True, 'restored', {
            'result': result,
            'expected': snapshot,
            'status': self.get_autopilot_setup_status(),
        }

    def ignore_autopilot_drift(self):
        """Persist ignore_drift=True so the drift banner never returns."""
        setup = self.state.get('autopilot_setup') or {}
        if not setup.get('applied'):
            return False, 'no setup applied yet', {}
        setup['ignore_drift'] = True
        self.state['autopilot_setup'] = setup
        self.save_state()
        self.app_logger.info(
            "ArduRover setup drift ignored for wind=SERIAL%s gps=SERIAL%s",
            setup.get('wind_serial'), setup.get('gps_serial'),
        )
        return True, 'ignored', {'status': self.get_autopilot_setup_status()}

    def get_mavlink_nvf_status(self):
        """Diagnostics for the NAMED_VALUE_FLOAT publisher, for the UI / SSE."""
        if not HAS_MAVLINK_SENDER or self._mav_sender is None:
            return {'enabled': False}
        return {
            'enabled': True,
            'interval_s': self._mav_publish_interval_s,
            'planned_component_ids': self._mav_sender.planned_component_ids(),
            'names': sorted(_MAV_NVF_OFFSETS.keys()),
            'count_sent': self._mav_publish_count,
            'last_ok_ts': self._mav_publish_last_ok_ts,
        }

    # ── mavlink2rest NAMED_VALUE_FLOAT publisher ─────────────────────
    #
    # Periodic background thread that lifts the latest parsed apparent / true
    # wind values out of `self.sensor_data` and POSTs them to mavlink2rest as
    # NAMED_VALUE_FLOAT messages. The autopilot logs every NVF it receives as
    # an `NVF` row in the DataFlash .BIN log (`WX_AppSpd`, `WX_AppDir`,
    # `WX_TruSpd`, `WX_TruDir`), which is what makes them line up against
    # GPS / ATT / WIND post-flight.
    #
    # Names are deliberately distinct from ArduPilot's own `AppWndSpd` /
    # `AppWndDir` NVFs (emitted by `AP_WindVane::send_wind()` whenever
    # wind-vane is enabled). Co-existing streams let us tell, post-flight,
    # whether the wind-vane subsystem ever saw the $WIMWV that this extension
    # was sending over UDP NMEA -- if `WX_AppSpd` is healthy but `AppWndSpd`
    # stays at 0, `WNDVN_TYPE` / `WNDVN_SPEED_TYPE` or the UDP NMEA route is
    # misconfigured on the autopilot side.
    #
    # The freshness check below guarantees we never replicate the
    # Downloads/00000230.BIN "32k zero rows" pattern: a value is only POSTed
    # when its parse timestamp is within DEFAULT_FRESH_S of now. Missing or
    # stale data is dropped silently.

    def _wind_nvf_snapshot(self):
        """Return {name: value} for fresh wind NVFs; never returns 0-as-stale."""
        if not HAS_MAVLINK_SENDER:
            return {}
        now = time.time()
        fresh = _MAV_DEFAULT_FRESH_S
        out = {}

        def _take(section_key, value_key, nvf_name):
            """Pull `value_key` from sensor_data[section_key] iff it is fresh."""
            section = self.sensor_data.get(section_key) or {}
            value = section.get(value_key)
            if value is None:
                return
            iso_ts = section.get('timestamp')
            if not iso_ts:
                return
            try:
                # `timestamp` is a naive ISO string written via
                # datetime.now().isoformat(); compare back in epoch seconds.
                dt = datetime.datetime.fromisoformat(iso_ts)
                age = now - dt.timestamp()
            except Exception:
                return
            if age > fresh:
                return
            out[nvf_name] = value

        # Apparent wind: angle is bow-relative (0..360), speed in knots.
        # Source: $WIMWV with R/Relative reference.
        _take('wind_apparent', 'angle', 'WX_AppDir')
        _take('wind_apparent', 'speed_kts', 'WX_AppSpd')

        # True wind: prefer `direction_true` (north-referenced, $WIMWD); fall
        # back to the bow-relative `angle` field if only $WIMWV-T is being
        # emitted. Speed in knots either way.
        true_section = self.sensor_data.get('wind_true') or {}
        if true_section.get('direction_true') is not None:
            _take('wind_true', 'direction_true', 'WX_TruDir')
        else:
            _take('wind_true', 'angle', 'WX_TruDir')
        _take('wind_true', 'speed_kts', 'WX_TruSpd')

        return out

    def _mav_publish_loop(self):
        """1 Hz NVF publisher. Never raises out of the thread."""
        # Stagger the first publish off the rest of the boot sequence so the
        # auto-connect, websocket server, and waitress all settle first.
        time.sleep(2.0)
        while not self.should_stop:
            try:
                values = self._wind_nvf_snapshot()
                if values and self._mav_sender is not None:
                    sent = self._mav_sender.send_named_value_floats(values)
                    if sent:
                        self._mav_publish_count += sent
                        self._mav_publish_last_ok_ts = time.time()
            except Exception as e:
                # Belt-and-braces: a bug in this loop must never take down
                # the rest of the extension.
                self.app_logger.error("mavlink2rest publisher error: %s", e)
            time.sleep(self._mav_publish_interval_s)

    def stream_message(self, message, msg_type):
        """Stream one NMEA sentence to the matching UDP route, if any.

        Each sentence is routed at most once — MWV goes to wind, and
        GGA/RMC/VTG/HDT go to GPS. Everything else is skipped. Bumps the
        matching per-route counter and the legacy `streamed_messages` total
        so existing counters keep counting.
        """
        if not self.is_streaming:
            return
        route = self._route_for_sentence(msg_type)
        if route is None:
            return
        port, counter_attr = route
        try:
            if not self.udp_socket:
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.app_logger.debug("Created new UDP socket")

            encoded_message = (message + '\n').encode()
            self.udp_socket.sendto(encoded_message, (self.UDP_HOST, port))
            setattr(self, counter_attr, getattr(self, counter_attr) + 1)
            self.streamed_messages += 1
        except Exception as e:
            self.app_logger.error(f"Error streaming message to {self.UDP_HOST}:{port}: {e}")
            try:
                if self.udp_socket:
                    self.udp_socket.close()
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except Exception as socket_error:
                self.app_logger.error(f"Failed to recreate socket: {socket_error}")

    # ── Cockpit data-lake WebSocket ──────────────────────────────────

    def _ws_process_request(self, connection, req):
        """Return HTTP 200 for non-WebSocket probes (BlueOS service scanner)."""
        if "Upgrade" not in req.headers:
            return WsResponse(200, "OK", WsHeaders(), b"Airmar 300WX WebSocket endpoint\n")

    def _start_ws_server(self):
        """Start the Cockpit data-lake WebSocket server on a background thread."""
        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._ws_loop = loop
            loop.run_until_complete(self._ws_serve())

        t = threading.Thread(target=_run, daemon=True, name='cockpit-ws')
        t.start()
        self.app_logger.info("Cockpit data-lake WebSocket server started on ws://0.0.0.0:8765")

    async def _ws_serve(self):
        async with websockets.serve(
            self._ws_handler, '0.0.0.0', 8765,
            process_request=self._ws_process_request,
        ):
            await asyncio.Future()

    async def _ws_handler(self, ws):
        self._ws_clients.add(ws)
        self.app_logger.debug(f"WS client connected: {ws.remote_address} ({len(self._ws_clients)} total)")
        try:
            await ws.send("wx300-connection-status=connected")
            async for _ in ws:
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._ws_clients.discard(ws)
            self.app_logger.debug(f"WS client disconnected ({len(self._ws_clients)} total)")

    def ws_broadcast(self, variable, value):
        """Send a variable=value pair to all connected Cockpit clients."""
        if not self._ws_clients or self._ws_loop is None:
            return
        msg = f"{variable}={value}"

        async def _send():
            dead = set()
            for client in self._ws_clients.copy():
                try:
                    await client.send(msg)
                except Exception:
                    dead.add(client)
            self._ws_clients.difference_update(dead)

        asyncio.run_coroutine_threadsafe(_send(), self._ws_loop)

    # ── Serial port discovery ─────────────────────────────────────

    def get_ports(self):
        """Get list of available serial ports"""
        ports = []
        # Check common USB serial ports
        for i in range(4):  # Check ttyUSB0 through ttyUSB3
            port = f'/dev/ttyUSB{i}'
            if Path(port).exists():
                ports.append(port)
        
        # Check common ACM ports
        for i in range(2):  # Check ttyACM0 through ttyACM1
            port = f'/dev/ttyAMA{i}'
            if Path(port).exists():
                ports.append(port)
        
        # Also check using pyserial's list_ports
        try:
            for port in serial.tools.list_ports.comports():
                if port.device not in ports:
                    ports.append(port.device)
        except Exception as e:
            self.app_logger.error(f"Error listing ports: {e}")
        
        return ports

    def get_device_ids(self):
        """Get mapping of /dev/serial/by-id/ names to their device paths with USB port info"""
        devices = []
        
        # First, build a map of device -> by-path info for USB port location
        by_path_map = {}
        try:
            serial_by_path = Path('/dev/serial/by-path')
            if serial_by_path.exists() and serial_by_path.is_dir():
                for link in serial_by_path.iterdir():
                    try:
                        if link.is_symlink():
                            real_device = str(link.resolve())
                            path_name = link.name
                            usb_port = self._parse_usb_port(path_name)
                            by_path_map[real_device] = {
                                'path_name': path_name,
                                'usb_port': usb_port
                            }
                    except Exception as e:
                        self.app_logger.error(f"Error reading by-path symlink {link}: {e}")
        except Exception as e:
            self.app_logger.error(f"Error reading /dev/serial/by-path: {e}")
        
        # Now read by-id and combine with by-path info
        try:
            serial_by_id = Path('/dev/serial/by-id')
            if serial_by_id.exists() and serial_by_id.is_dir():
                for link in serial_by_id.iterdir():
                    try:
                        if link.is_symlink():
                            real_device = str(link.resolve())
                            by_id_name = link.name
                            # Clean up name for display
                            display_name = by_id_name.replace('usb-', '').replace('-if00-port0', '').replace('_', ' ')
                            
                            # Get USB port info from by-path map
                            path_info = by_path_map.get(real_device, {})
                            usb_port = path_info.get('usb_port', {'position': 'unknown', 'label': 'Unknown', 'type': 'unknown'})
                            
                            devices.append({
                                'device': real_device,
                                'by_id_name': by_id_name,
                                'display_name': display_name,
                                'usb_port': usb_port,
                                'path_name': path_info.get('path_name', '')
                            })
                    except Exception as e:
                        self.app_logger.error(f"Error reading symlink {link}: {e}")
        except Exception as e:
            self.app_logger.error(f"Error reading /dev/serial/by-id: {e}")
        
        # Sort by device name for consistent ordering
        devices.sort(key=lambda x: x['device'])
        return devices

    # BlueOS-style path-to-position map (PR 3403). Keys are substrings to match in the
    # normalized by-path name; sorted longest-first so e.g. 1.1.3 matches before 1.3.
    # Layout: top-left, top-right / bottom-left, bottom-right (ethernet on left).
    _USB_PATH_POSITION_MAP = sorted([
        # Pi4 - full path keys (BlueOS convention)
        ('platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3', 'top-left'),
        ('platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4', 'bottom-left'),
        ('platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1', 'top-right'),
        ('platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2', 'bottom-right'),
        # Pi4 - kernel 1.1.x style (must match before single 1.1 / 1.2 / 1.3)
        ('-usb-0:1.1.3', 'top-left'),
        ('-usb-0:1.1.2', 'bottom-left'),
        ('-usb-0:1.1.4', 'bottom-left'),
        ('-usb-0:1.3:', 'top-right'),
        ('-usb-0:1.3-', 'top-right'),
        ('-usb-0:1.2:', 'bottom-right'),
        ('-usb-0:1.2-', 'bottom-right'),
        ('-usb-0:1.4', 'bottom-left'),
        ('-usb-0:1.1', 'top-right'),
        # Pi3
        ('platform-3f980000.usb-usb-0:1.5:1', 'bottom-right'),
        ('platform-3f980000.usb-usb-0:1.4:1', 'top-right'),
        ('platform-3f980000.usb-usb-0:1.3:1', 'bottom-left'),
        ('platform-3f980000.usb-usb-0:1.2:1', 'top-left'),
        # Pi5
        ('platform-xhci-hcd.1-usb-0:2', 'bottom-right'),
        ('platform-xhci-hcd.0-usb-0:2', 'top-right'),
        ('platform-xhci-hcd.1-usb-0:1', 'top-left'),
        ('platform-xhci-hcd.0-usb-0:1', 'bottom-left'),
    ], key=lambda p: -len(p[0]))

    def _parse_usb_port(self, path_name):
        """
        Parse the by-path symlink name to physical USB port position (BlueOS-style).
        Uses prefix/contains matching so path format variations still resolve.
        Detects hub connections and returns overlay info when present.
        """
        try:
            import re
            # Normalize like BlueOS: strip -port0 so we match on the USB root
            usb_root = path_name.split('-port0')[0] if path_name else ''
            # Extract bus path for display (e.g. 0:1.1.3)
            bus_match = re.search(r'usb-(\d+:\d+(?:\.\d+)*)', path_name or '')
            bus_path = bus_match.group(1) if bus_match else (usb_root or '')

            # Hub detection (BlueOS overlay): ...usb-0:1.4.3:1.0... -> "hub port 3"
            hub_info = None
            hub_match = re.search(r'usb-0:(?:[0-9]+\.)+([0-9]+):1\.0', path_name or '')
            if hub_match:
                hub_info = f"Via hub, port {hub_match.group(1)}"

            # First matching key wins (map is longest-first for specificity)
            for key, position in self._USB_PATH_POSITION_MAP:
                if key in usb_root:
                    label = position.replace('-', ' ').title()
                    result = {
                        'position': position,
                        'label': label,
                        'type': 'usb3' if 'right' in position else 'usb2',  # Pi4: right = USB3
                        'bus': bus_path,
                    }
                    if hub_info:
                        result['hub_info'] = hub_info
                    self.app_logger.debug(f"USB path: {path_name} -> {position} ({bus_path})")
                    return result

            self.app_logger.debug(f"USB path: {path_name} -> no match (usb_root={usb_root})")
            result = {'position': 'unknown', 'label': 'Unknown', 'type': 'unknown', 'bus': bus_path}
            if hub_info:
                result['hub_info'] = hub_info
            return result
        except Exception as e:
            self.app_logger.error(f"Error parsing USB port from {path_name}: {e}")
            return {'position': 'unknown', 'label': 'Unknown', 'type': 'unknown', 'bus': ''}

    def _safe_serial_open(self, port, baud_rate, open_timeout=5):
        """Open a serial port with a timeout guard against kernel-level hangs.
        Returns a serial.Serial connection or raises on failure."""
        result = [None, None]
        def _open():
            try:
                result[0] = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
            except Exception as e:
                result[1] = e
        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=open_timeout)
        if t.is_alive():
            raise serial.SerialException(
                f"serial open hung for {open_timeout}s on {port} at {baud_rate}")
        if result[1] is not None:
            raise result[1]
        return result[0]

    def _try_baud_rate(self, port, baud_rate, timeout=3):
        """
        Try to connect at a specific baud rate and wait for valid NMEA data.
        Sends enable periodic ($PAMTX,1*hh) as soon as the port is open so a stopped
        device can start sending before we check for incoming data.
        At 4800 baud, output may be ~1 Hz per sentence; require fewer lines but a
        longer window than at the high operating baud.
        Returns (success, serial_connection or None)
        """
        min_messages = 2 if baud_rate == 4800 else 1
        try:
            self.app_logger.debug(f"Trying {port} at {baud_rate} baud...")
            conn = self._safe_serial_open(port, baud_rate)
            conn.reset_input_buffer()
            conn.write(self._nmea_cmd('PAMTX,1'))
            time.sleep(0.3)

            start_time = time.time()
            valid_count = 0
            while time.time() - start_time < timeout:
                try:
                    data = conn.readline().decode('utf-8', errors='ignore').strip()
                    if self._incoming_line_checksum_valid(data):
                        valid_count += 1
                        addr = data.split(',')[0].lstrip('$!').strip()
                        msg_type = self._nmea_sentence_formatter(addr)
                        self.app_logger.debug(f"NMEA at {baud_rate}: {msg_type} ({valid_count}/{min_messages})")
                        if valid_count >= min_messages:
                            return True, conn
                except Exception as e:
                    self.app_logger.error(f"Error reading at {baud_rate} baud: {e}")
                time.sleep(0.1)

            conn.close()
            return False, None

        except Exception as e:
            self.app_logger.error(f"Failed to open port at {baud_rate} baud: {e}")
            return False, None

    def _switch_to_operating_baud(self, port):
        """
        Switch the device from 4800 to OPERATING_BAUD_RATE (115200).
        Assumes we're currently connected at 4800 baud.
        Sends $PAMTC,BAUD,<rate>,CFG after the switch so the default rate is stored in NVM
        (applies from next power-on per 300WX manual).
        Returns (success, message)
        """
        rate = self.OPERATING_BAUD_RATE
        try:
            self._set_conn_status(self.CONN_STATUS_SWITCHING_BAUD,
                f'{port} — switching to {rate} baud')

            # Per 300WX manual: suspend transmission first, then change baud, then resume.

            # Step 1: $PAMTX,0 — suspend periodic sentences at 4800
            self.app_logger.debug("$PAMTX,0 — suspend transmission at 4800")
            self.serial_connection.write(self._nmea_cmd('PAMTX,0'))
            time.sleep(0.3)

            # Step 2: $PAMTC,BAUD,<rate>*hh at 4800 — immediate switch on the wire
            self.app_logger.debug(f"$PAMTC,BAUD,{rate} at 4800")
            self.serial_connection.write(self._nmea_cmd(f'PAMTC,BAUD,{rate}'))
            time.sleep(0.5)

            # Step 3: Drain any remaining bytes at 4800
            self.app_logger.debug("Draining 4800 baud output...")
            self.serial_connection.timeout = 0.2
            drain_deadline = time.time() + 1.0
            while time.time() < drain_deadline:
                raw = self.serial_connection.readline()
                if not raw:
                    break
            self.serial_connection.timeout = 1

            # Step 4: Host reopen at operating baud
            self.serial_connection.close()
            time.sleep(0.3)
            self.app_logger.debug(f"Reopening at {rate} baud")
            self.serial_connection = self._safe_serial_open(port, rate)
            self.state['baud_rate'] = rate
            time.sleep(0.2)

            # Step 5: Resume periodic output
            self.app_logger.debug(f"$PAMTX,1 — resume transmission at {rate}")
            self.serial_connection.write(self._nmea_cmd('PAMTX,1'))
            time.sleep(0.3)

            # Step 6: Store default baud in NVM (does not change active rate; next cold boot uses this)
            self.app_logger.debug(f"$PAMTC,BAUD,{rate},CFG — save default baud to NVM")
            self.serial_connection.write(self._nmea_cmd(f'PAMTC,BAUD,{rate},CFG'))
            time.sleep(0.3)

            # Step 7: Verify we're receiving valid NMEA at operating baud
            start_time = time.time()
            while time.time() - start_time < 5:
                data = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                if self._incoming_line_checksum_valid(data):
                    self.app_logger.debug(f"Confirmed communication at {rate}")
                    return True, f"Switched to {rate} baud (default saved with CFG)"
                time.sleep(0.1)

            return False, "No response after baud rate switch"
            
        except Exception as e:
            self.app_logger.error(f"Error switching baud rate: {e}")
            return False, str(e)

    def _required_interval_for(self, sentence_id, baud_rate):
        """Return the PAMTC,EN interval (tenths-of-seconds) for one sentence.

        At the operating baud (115200), GPS sentences that feed the
        ArduPilot NMEA GPS driver are stepped up to 10 Hz so EKF3's GPS
        source stays responsive. Everything else uses the class-dict
        `default_interval` (1 Hz). At 4800 baud all sentences fall back
        to 1 Hz to stay under the bus ceiling.
        """
        config = self.SUPPORTED_SENTENCES.get(sentence_id, {})
        default_tenths = int(config.get('default_interval', 10))
        if baud_rate == self.OPERATING_BAUD_RATE:
            override = self.HIGH_BAUD_GPS_HZ_OVERRIDE_TENTHS.get(sentence_id)
            if override is not None:
                return int(override)
        return default_tenths

    def enable_required_sentences(self):
        """
        Enable the NMEA sentences required for dashboard display and for
        ArduPilot GPS ingestion. Called after connect at either 4800 or
        115200 baud. Always enables periodic transmission first in case
        the device was previously stopped. Saved per-sentence overrides
        (from `state.json:sentence_config`) are applied AFTER this call
        by the connect path, so explicit user settings take precedence.

        Returns (success, message)
        """
        try:
            port_label = self.serial_connection.port if self.serial_connection else '?'
            self._set_conn_status(self.CONN_STATUS_ENABLING_SENTENCES,
                                  f'{port_label} — enabling NMEA sentences')
            
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected"
            
            # Ensure periodic sentences are on (device may have been stopped by a previous session)
            self.serial_connection.write(self._nmea_cmd('PAMTX,1'))
            time.sleep(0.3)

            baud = self.serial_connection.baudrate
            enabled_count = 0
            fast_ids = []
            for sentence_id in self.REQUIRED_SENTENCES:
                interval = self._required_interval_for(sentence_id, baud)
                cmd = self._nmea_cmd(f'PAMTC,EN,{sentence_id},1,{interval}')
                self.app_logger.debug(
                    "Enabling sentence %s at %d/10 s (%.1f Hz)",
                    sentence_id, interval, 10.0 / max(interval, 1),
                )
                self.serial_connection.write(cmd)
                time.sleep(0.2)
                enabled_count += 1
                if interval == 1:
                    fast_ids.append(sentence_id)

            if fast_ids:
                self.app_logger.info(
                    "Enabled %d required sentences (10 Hz: %s @ %d baud)",
                    enabled_count, ",".join(sorted(fast_ids)), baud,
                )
            else:
                self.app_logger.info(
                    "Enabled %d required sentences at 1 Hz (@ %d baud)",
                    enabled_count, baud,
                )
            return True, f"Enabled {enabled_count} sentences"
            
        except Exception as e:
            self.app_logger.error(f"Error enabling sentences: {e}")
            return False, str(e)

    def configure_sentence(self, sentence_id, enabled, interval=None):
        """
        Configure a single NMEA sentence on the device.
        
        Args:
            sentence_id: The sentence identifier (e.g., 'MWD', 'MWVR')
            enabled: True to enable, False to disable
            interval: Transmission interval in tenths of seconds (default: 10 = 1 second)
        
        Returns (success, message)
        """
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected"
            
            if sentence_id not in self.SUPPORTED_SENTENCES:
                return False, f"Unknown sentence: {sentence_id}"
            
            if interval is None:
                interval = self.SUPPORTED_SENTENCES[sentence_id].get('default_interval', 10)
            
            enable_flag = 1 if enabled else 0
            cmd = self._nmea_cmd(f'PAMTC,EN,{sentence_id},{enable_flag},{interval}')

            self.app_logger.debug(f"Configuring sentence {sentence_id}: enabled={enabled}, interval={interval/10}s")
            with self._serial_lock:
                self.serial_connection.write(cmd)
                time.sleep(0.2)
            
            if 'sentence_config' not in self.state:
                self.state['sentence_config'] = {}
            self.state['sentence_config'][sentence_id] = {'enabled': enabled, 'interval': interval}
            self.save_state()
            action = "Enabled" if enabled else "Disabled"
            return True, f"{action} {sentence_id}"
            
        except Exception as e:
            self.app_logger.error(f"Error configuring sentence {sentence_id}: {e}")
            return False, str(e)

    def configure_sentences_batch(self, changes):
        """Configure multiple sentences in one locked session.
        changes: list of dicts: {sentence_id, enabled, interval} where interval is tenths-of-seconds or None."""
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected"
            if not changes:
                return True, "No changes to apply"
            errors = []
            applied = 0
            # #region agent log
            try:
                _dbg_path = Path(__file__).resolve().parent.parent / '.cursor' / 'debug.log'
                _dbg_path.parent.mkdir(parents=True, exist_ok=True)
                with open(_dbg_path, 'a') as _f:
                    _f.write(json.dumps({'id': 'batch_lock_enter', 'timestamp': time.time(), 'location': 'main.py:configure_sentences_batch', 'message': 'batch lock enter', 'data': {'n_changes': len(changes)}, 'hypothesisId': 'H3'}) + '\n')
            except Exception:
                pass
            # #endregion
            with self._serial_lock:
                for ch in changes:
                    sentence_id = ch.get('sentence_id')
                    enabled = bool(ch.get('enabled', True))
                    interval = ch.get('interval', None)
                    if not sentence_id or sentence_id not in self.SUPPORTED_SENTENCES:
                        errors.append(f"{sentence_id or '(missing id)'}: unknown sentence")
                        continue
                    if interval is None:
                        interval = self.SUPPORTED_SENTENCES[sentence_id].get('default_interval', 10)
                    enable_flag = 1 if enabled else 0
                    cmd = self._nmea_cmd(f'PAMTC,EN,{sentence_id},{enable_flag},{interval}')
                    try:
                        self.serial_connection.write(cmd)
                        if 'sentence_config' not in self.state:
                            self.state['sentence_config'] = {}
                        self.state['sentence_config'][sentence_id] = {'enabled': enabled, 'interval': interval}
                        applied += 1
                        time.sleep(0.15)
                    except Exception as e:
                        errors.append(f"{sentence_id}: {e}")
            # #region agent log
            try:
                _dbg_path = Path(__file__).resolve().parent.parent / '.cursor' / 'debug.log'
                with open(_dbg_path, 'a') as _f:
                    _f.write(json.dumps({'id': 'batch_lock_exit', 'timestamp': time.time(), 'location': 'main.py:configure_sentences_batch', 'message': 'batch lock exit', 'data': {'applied': applied, 'n_changes': len(changes)}, 'hypothesisId': 'H3'}) + '\n')
            except Exception:
                pass
            # #endregion
            if applied > 0:
                self.save_state()
            if errors:
                self.app_logger.warning(f"Batch configure had errors: {errors}")
                return False, f"Applied {applied}/{len(changes)} changes; errors: " + "; ".join(errors[:3])
            return True, f"Applied {applied} sentence changes"
        except Exception as e:
            self.app_logger.error(f"Error batch configuring sentences: {e}")
            return False, str(e)

    def _classify_wx_model(self, model_code):
        """
        Classify a WX model code reported via $PAMTR,QPS into a family label.
        Returns dict with keys: family ('300WX'|'200WX'|'150WX'|'100WX'|'unknown'),
        is_legacy (True for models that predate the 300WX and cannot run at 115200).
        """
        code = (model_code or '').upper().strip()
        if code.startswith('300WX'):
            return {'family': '300WX', 'is_legacy': False}
        if code.startswith('200WX'):
            return {'family': '200WX', 'is_legacy': True}
        if code.startswith('150WX'):
            return {'family': '150WX', 'is_legacy': True}
        if code.startswith('100WX'):
            return {'family': '100WX', 'is_legacy': True}
        return {'family': 'unknown', 'is_legacy': False}

    def query_device_info(self, timeout=2.0):
        """
        Identify the connected device by sending $PAMTC,QPS (part/serial/model)
        and $PAMTC,QV (hardware/firmware versions) and parsing the responses.

        Also accepts $PAMTT,QPS/$PAMTT,QV lines which the device emits once on
        power-up per the WX manual. Updates self.device_info and returns it,
        or None on failure. Must be called while the reader thread is not yet
        running so responses are not consumed elsewhere.
        """
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return None

            info = {
                'model_code': None,
                'part_number': None,
                'serial_number': None,
                'hardware_version': None,
                'firmware_version': None,
                'family': 'unknown',
                'is_legacy': False,
            }

            with self._serial_lock:
                try:
                    self.serial_connection.reset_input_buffer()
                except Exception:
                    pass
                self.serial_connection.write(self._nmea_cmd('PAMTC,QPS'))
                time.sleep(0.1)
                self.serial_connection.write(self._nmea_cmd('PAMTC,QV'))

                deadline = time.time() + timeout
                prev_timeout = self.serial_connection.timeout
                self.serial_connection.timeout = 0.2
                try:
                    while time.time() < deadline:
                        try:
                            line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                        except Exception as read_err:
                            if 'returned no data' in str(read_err) or 'multiple access' in str(read_err):
                                time.sleep(0.05)
                                continue
                            break
                        if not line:
                            continue
                        if not (line.startswith('$PAMTR,') or line.startswith('$PAMTT,')):
                            continue
                        if not self._incoming_line_checksum_valid(line):
                            continue
                        body = line.split('*', 1)[0]
                        parts = body.split(',')
                        if len(parts) < 2:
                            continue
                        tag = parts[1]
                        if tag == 'QPS' and len(parts) >= 5:
                            info['part_number'] = parts[2] or None
                            info['serial_number'] = parts[3] or None
                            info['model_code'] = parts[4] or None
                        elif tag == 'QV' and len(parts) >= 7:
                            info['hardware_version'] = parts[3] or None
                            info['firmware_version'] = parts[6] or None
                        if info['model_code'] and info['firmware_version']:
                            break
                finally:
                    try:
                        self.serial_connection.timeout = prev_timeout
                    except Exception:
                        pass

            if not info['model_code']:
                self.app_logger.debug('Device identification: no $PAMTR,QPS response received')
                self.device_info = None
                return None

            classified = self._classify_wx_model(info['model_code'])
            info.update(classified)
            self.device_info = info
            self.app_logger.info(
                f"Device identified: model={info['model_code']} "
                f"(family {info['family']}, legacy={info['is_legacy']}) "
                f"hw={info['hardware_version']} fw={info['firmware_version']} "
                f"serial={info['serial_number']}"
            )
            return info
        except Exception as e:
            self.app_logger.warning(f"Device identification failed: {e}")
            return None

    def query_sentence_config(self):
        """
        Query the device for current sentence configuration.
        Sends $PAMTC,EN,Q and parses the response.
        Holds the serial read lock so the reader thread does not consume response lines.
        
        Returns (success, config_dict or error_message)
        """
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected"
            
            # #region agent log
            try:
                _dbg_path = Path(__file__).resolve().parent.parent / '.cursor' / 'debug.log'
                _dbg_path.parent.mkdir(parents=True, exist_ok=True)
                with open(_dbg_path, 'a') as _f:
                    _f.write(json.dumps({'id': 'query_lock_enter', 'timestamp': time.time(), 'location': 'main.py:query_sentence_config', 'message': 'query lock enter (will hold up to 5s)', 'data': {}, 'hypothesisId': 'H4'}) + '\n')
            except Exception:
                pass
            # #endregion
            with self._serial_lock:
                # Clear any pending data
                self.serial_connection.reset_input_buffer()
                
                # Send query command
                self.app_logger.debug("Querying device sentence configuration")
                self.serial_connection.write(self._nmea_cmd('PAMTC,EN,Q'))
                
                # Collect responses (device sends multiple $PAMTR,EN lines)
                config = {}
                start_time = time.time()
                timeout = 5  # 5 second timeout
                
                while time.time() - start_time < timeout:
                    try:
                        line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    except Exception as read_err:
                        # USB serial often raises "returned no data" spuriously; retry
                        if "returned no data" in str(read_err) or "multiple access" in str(read_err):
                            time.sleep(0.05)
                            continue
                        raise
                    if not line:
                        time.sleep(0.05)
                        continue
                    if line.startswith('$PAMTR,EN,'):
                        # Parse: $PAMTR,EN,<total>,<num>,<id>,<enabled>,<interval> or $PAMTR,EN,<id>,<enabled>,<interval>
                        # Strip the NMEA `*XX` checksum first — otherwise
                        # the last field arrives as e.g. `1*45`, `isdigit()`
                        # returns False, and every interval silently
                        # defaults to 10 (1 Hz). This mis-reported 10 Hz
                        # sentences as 1 Hz in the UI even though the
                        # device was already running at the higher rate.
                        line_body = line.split('*', 1)[0]
                        parts = line_body.split(',')
                        if len(parts) >= 6:
                            # If 7+ parts: total,num,id,enabled,interval at 2,3,4,5,6
                            if len(parts) >= 7 and parts[4] in self.SUPPORTED_SENTENCES:
                                sentence_id, enabled_str, interval_str = parts[4], parts[5], parts[6]
                            else:
                                sentence_id, enabled_str, interval_str = parts[3], parts[4], parts[5]
                            enabled = enabled_str == '1'
                            interval_str = interval_str.strip()
                            interval = int(interval_str) if interval_str.isdigit() else 10
                            if sentence_id in self.SUPPORTED_SENTENCES:
                                config[sentence_id] = {'enabled': enabled, 'interval': interval}
                                self.app_logger.debug(f"  {sentence_id}: enabled={enabled}, interval={interval/10}s")
                    
                    if len(config) >= len(self.SUPPORTED_SENTENCES):
                        break
                    time.sleep(0.05)
            # #region agent log
            try:
                _dbg_path = Path(__file__).resolve().parent.parent / '.cursor' / 'debug.log'
                with open(_dbg_path, 'a') as _f:
                    _f.write(json.dumps({'id': 'query_lock_exit', 'timestamp': time.time(), 'location': 'main.py:query_sentence_config', 'message': 'query lock exit', 'data': {'config_count': len(config)}, 'hypothesisId': 'H4'}) + '\n')
            except Exception:
                pass
            # #endregion
            if config:
                return True, config
            else:
                return False, "No configuration response received"
            
        except Exception as e:
            self.app_logger.error(f"Error querying sentence config: {e}")
            return False, str(e)

    def save_sentence_config(self):
        """
        Save current sentence configuration to device EEPROM.
        Sends $PAMTC,EN,S command.
        
        Returns (success, message)
        """
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected"
            
            self.app_logger.debug("Saving sentence configuration to device EEPROM")
            self.serial_connection.write(self._nmea_cmd('PAMTC,EN,S'))
            time.sleep(0.5)
            
            return True, "Configuration saved to device EEPROM"
            
        except Exception as e:
            self.app_logger.error(f"Error saving sentence config: {e}")
            return False, str(e)

    def load_sentence_defaults(self):
        """
        Load factory default sentence configuration from device ROM to RAM.
        Sends $PAMTC,EN,LD command.
        
        Returns (success, message)
        """
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected"
            
            self.app_logger.debug("Loading factory default sentence configuration")
            self.serial_connection.write(self._nmea_cmd('PAMTC,EN,LD'))
            time.sleep(0.5)
            
            return True, "Factory defaults loaded"
            
        except Exception as e:
            self.app_logger.error(f"Error loading sentence defaults: {e}")
            return False, str(e)

    def get_sentences_info(self):
        """
        Get information about all supported sentences.
        Returns list of sentence info dictionaries.
        """
        sentences = []
        for sentence_id, config in self.SUPPORTED_SENTENCES.items():
            sentences.append({
                'id': sentence_id,
                'name': config['name'],
                'description': config['description'],
                'default_enabled': config['default_enabled'],
                'default_interval': config['default_interval'],
                'max_chars': config.get('max_chars', 82),
                'required': sentence_id in self.REQUIRED_SENTENCES
            })
        
        # Sort by ID
        sentences.sort(key=lambda x: x['id'])
        return sentences

    def connect_serial(self, port, baud_rate=None, stay_at_4800=False,
                        max_attempts=6):
        """
        Connect to serial port with automatic baud rate negotiation.

        Args:
            port: Serial port path
            baud_rate: Hint for which baud to try first (saved state)
            stay_at_4800: If True, never switch to OPERATING_BAUD_RATE
            max_attempts: Total attempts across all baud rates (default 6 = 3 per baud)
        """
        try:
            # Clean up any existing connection
            if self.serial_connection and self.serial_connection.is_open:
                self.stop_reader_thread()
                self.serial_connection.close()
                self.serial_connection = None

            if not Path(port).exists():
                self._set_conn_status(self.CONN_STATUS_FAILED, f"Port {port} does not exist")
                return False, self.connection_message

            self.detected_baud = None
            op = self.OPERATING_BAUD_RATE
            if stay_at_4800:
                baud_rates = [4800]
            elif baud_rate == op:
                baud_rates = [op, 4800]
            else:
                baud_rates = [4800, op]
            
            # Try to establish connection
            n_bauds = len(baud_rates)
            for attempt in range(max_attempts):
                if self._cancel_connect:
                    self._set_conn_status(self.CONN_STATUS_DISCONNECTED, 'Cancelled')
                    return False, 'Cancelled'

                current_baud = baud_rates[attempt % n_bauds]
                attempt_num = attempt // n_bauds + 1
                max_per_baud = max_attempts // n_bauds

                status = (self.CONN_STATUS_TRYING_4800 if current_baud == 4800
                          else self.CONN_STATUS_TRYING_HIGH_BAUD)
                self._set_conn_status(status, f'{port} — {current_baud} baud (attempt {attempt_num}/{max_per_baud})')

                self.app_logger.debug(self.connection_message)

                probe_timeout = 12.0 if current_baud == 4800 else 4.0
                success, conn = self._try_baud_rate(port, current_baud, timeout=probe_timeout)
                
                if success:
                    self.serial_connection = conn
                    self.detected_baud = current_baud
                    self.state['port'] = port
                    
                    if current_baud == 4800 and not stay_at_4800:
                        # Connected at 4800, switch to operating baud unless user requested 4800 only
                        self.app_logger.debug(f"Connected at 4800, switching to {op}...")
                        success, msg = self._switch_to_operating_baud(port)
                        if not success:
                            self.app_logger.warning(
                                f"Switch to {op} failed: {msg}; will try {op} directly")
                            if self.serial_connection and self.serial_connection.is_open:
                                self.serial_connection.close()
                            self.serial_connection = None
                            continue
                    elif current_baud == 4800 and stay_at_4800:
                        self.app_logger.debug("Staying at 4800 (no baud switch)")
                    elif current_baud == op and stay_at_4800:
                        # User asked for 4800 only but device is at high baud; move link to 4800
                        self.app_logger.debug(
                            f"Switching device from {op} to 4800 (user requested 4800 only)")
                        success, msg = self.change_baud_rate(4800)
                        if not success:
                            self.app_logger.warning(f"Could not switch device to 4800: {msg}")
                        # Continue; reader will be started in common path below
                    
                    # Identify the device (model / firmware) before starting the
                    # reader thread so $PAMTR,QPS/QV responses are not consumed
                    # by the async reader. Failure is non-fatal.
                    try:
                        self.query_device_info()
                    except Exception as _qe:
                        self.app_logger.debug(f"query_device_info error: {_qe}")

                    # Enable required sentences (4800 or operating baud)
                    success, msg = self.enable_required_sentences()
                    if not success:
                        self.app_logger.warning(f"Failed to enable sentences: {msg}")
                        # Continue anyway - sentences may already be enabled
                    # Apply saved sentence config so device matches last UI state after restart
                    saved = self.state.get('sentence_config') or {}
                    if saved:
                        changes = [{'sentence_id': sid, 'enabled': c['enabled'], 'interval': c.get('interval')}
                                  for sid, c in saved.items() if sid in self.SUPPORTED_SENTENCES]
                        if changes:
                            apply_ok, _ = self.configure_sentences_batch(changes)
                            if apply_ok:
                                self.app_logger.debug("Restored saved sentence config to device")
                    
                    # Start the reader thread
                    self.start_reader_thread()
                    
                    # Update state from actual serial baud (e.g. 4800 after change_baud_rate(4800))
                    final_baud = self.serial_connection.baudrate if self.serial_connection else op
                    self.state['baud_rate'] = final_baud
                    self.state['stay_at_4800'] = bool(stay_at_4800)
                    self.save_state()
                    self.connected_since = time.time()
                    
                    if final_baud == 4800:
                        detected_msg = " (4800 only, no switch)"
                    elif self.detected_baud == op:
                        detected_msg = f" (detected at {op})"
                    else:
                        detected_msg = " (switched from 4800)"
                    self._set_conn_status(self.CONN_STATUS_CONNECTED,
                                          f"Connected to {port} at {final_baud} baud{detected_msg}")
                    self.app_logger.info(self.connection_message)
                    self._sse_broadcast('connection', self.get_connection_info())
                    
                    # Auto-start streaming WIMWV to autopilot on connect
                    self.start_streaming()
                    
                    return True, self.connection_message
            
            # All attempts failed
            self._set_conn_status(self.CONN_STATUS_FAILED,
                                  "Could not establish connection after multiple attempts")
            self.app_logger.error(self.connection_message)
            return False, self.connection_message

        except Exception as e:
            self._set_conn_status(self.CONN_STATUS_FAILED, str(e))
            self.app_logger.error(f"Error in connect_serial: {e}")
            return False, self.connection_message

    def disconnect_serial(self):
        """Disconnect from serial port"""
        try:
            # Stop streaming first
            if self.is_streaming:
                self.stop_streaming()
            
            # Stop the reader thread
            self.stop_reader_thread()
            
            # Close the serial connection
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
            
            self.serial_connection = None
            
            # Reset state
            self.state['port'] = None
            self.message_history = []  # Clear message history
            self.nmea_messages = set()  # Clear detected message types
            self.sentence_last_seen = {}
            self.connected_since = None
            self.messages_received = 0
            self.detected_baud = None
            self.device_info = None
            self._set_conn_status(self.CONN_STATUS_DISCONNECTED, '')
            self.save_state()
            self._sse_broadcast('connection', self.get_connection_info())
            
            return True, "Disconnected from serial port"
        except Exception as e:
            self.app_logger.error(f"Error disconnecting: {e}")
            return False, str(e)
    
    def get_connection_info(self):
        """Get detailed connection information"""
        is_connected = (self.serial_connection is not None and 
                       self.serial_connection.is_open)
        
        step_elapsed = None
        if self.connection_step_since is not None:
            step_elapsed = round(time.time() - self.connection_step_since, 1)

        return {
            'connected': is_connected,
            'status': self.connection_status,
            'message': self.connection_message,
            'step_elapsed': step_elapsed,
            'port': self.serial_connection.port if is_connected else None,
            'baud_rate': self.serial_connection.baudrate if is_connected else 0,
            'detected_baud': self.detected_baud,
            'required_sentences': list(self.REQUIRED_SENTENCES),
            'stay_at_4800': bool(self.state.get('stay_at_4800', False)),
            'saved_port': self.state.get('port'),
            'saved_baud_rate': self.state.get('baud_rate', 4800),
            'device_info': self.device_info if is_connected else None,
        }

    def read_serial(self):
        """Read data from serial port - returns filtered message history"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return {"status": "error", "message": "Not connected"}
        
        return {
            "status": "success",
            "messages": self.message_history,
            "now": time.time(),
            "connected_since": self.connected_since,
            "observed_sentence_last_seen": self.sentence_last_seen
        }

    def log_message(self, message):
        """Log one NMEA sentence to the rotating NMEA log.

        Previously this wrote the message twice: once directly to
        `self.log_path` with a manually-formatted timestamp, and once via
        `self.nmea_logger.info(message)`, whose FileHandler wrote to the
        same file. The result was every sentence appearing twice in
        `nmea_messages.log` — visible in the live vehicle output — and
        also broke rotation: when the rotator truncated the file, the
        direct `open('a')` call kept appending to the same inode after
        the rename, silently defeating the size cap.

        The rotating handler (see `__init__`) is now the sole writer.
        """
        try:
            self.nmea_logger.info(message)
            return True, "Message logged"
        except Exception as e:
            self.app_logger.error(f"Error logging NMEA message: {e}")
            return False, str(e)

    def change_baud_rate(self, new_baud_rate):
        """Change the baud rate of the weather station and save default to NVM (CFG)."""
        try:
            if not self.serial_connection or not self.serial_connection.is_open:
                return False, "Not connected to serial port"

            current_baud = self.serial_connection.baudrate
            if current_baud == new_baud_rate:
                return False, f"Already at {new_baud_rate} baud"

            # Step 1: Disable periodic sentences
            self.app_logger.debug("Disabling periodic sentences")
            self.serial_connection.write(self._nmea_cmd('PAMTX,0'))
            time.sleep(0.5)  # Wait for command to be processed

            # Step 2: Send baud rate change command
            self.app_logger.debug(f"Sending baud change to {new_baud_rate}")
            self.serial_connection.write(self._nmea_cmd(f'PAMTC,BAUD,{new_baud_rate}'))
            time.sleep(1)  # Wait for any queued messages

            # Step 3: Close current connection
            self.app_logger.debug("Closing current connection")
            self.serial_connection.close()
            time.sleep(0.5)  # Wait for port to fully close

            # Step 4: Reopen at new baud rate
            self.app_logger.debug(f"Reopening at {new_baud_rate} baud")
            self.serial_connection = self._safe_serial_open(self.state['port'], new_baud_rate)
            time.sleep(0.5)  # Wait for connection to stabilize

            # Step 5: Re-enable periodic sentences
            self.app_logger.debug("Re-enabling periodic sentences")
            self.serial_connection.write(self._nmea_cmd('PAMTX,1'))
            time.sleep(0.2)

            # Step 6: Persist as power-on default (300WX: $PAMTC,BAUD,<n>,CFG)
            self.app_logger.debug(f"Saving default baud {new_baud_rate} to NVM (CFG)")
            self.serial_connection.write(self._nmea_cmd(f'PAMTC,BAUD,{new_baud_rate},CFG'))

            # Update state
            self.state['baud_rate'] = new_baud_rate
            self.save_state()

            return True, f"Successfully changed baud rate to {new_baud_rate} (saved as default)"
        except Exception as e:
            self.app_logger.error(f"Error changing baud rate: {e}")
            return False, str(e)

# Create NMEA handler instance
nmea_handler = NMEAHandler()

def _sse_encode(event_name, data_obj):
    payload = json.dumps(data_obj, separators=(',', ':'))
    return f"event: {event_name}\ndata: {payload}\n\n"

@app.route('/api/events', methods=['GET'])
def sse_events():
    """Server-Sent Events stream for near real-time UI updates."""
    q = nmea_handler.sse_add_client()

    def gen():
        try:
            # Initial snapshot
            init = {
                'connection': nmea_handler.get_connection_info(),
                'stream_status': nmea_handler._stream_status_snapshot(),
                'sensor_data': nmea_handler.sensor_data,
                'messages': nmea_handler.message_history[:50],
            }
            yield _sse_encode('init', init)

            while True:
                try:
                    msg = q.get(timeout=15)
                    yield _sse_encode(msg['event'], msg['data'])
                except queue.Empty:
                    # Keepalive comment
                    yield ": keepalive\n\n"
        except GeneratorExit:
            pass
        finally:
            nmea_handler.sse_remove_client(q)

    headers = {
        "Cache-Control": "no-cache",
        "X-Accel-Buffering": "no",
    }
    return Response(gen(), mimetype='text/event-stream', headers=headers)

@app.route('/')
def index():
    """Serve the main page"""
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/widget')
def widget_page():
    """Narrow wind + navigation widget (embeddable)"""
    return send_from_directory(app.static_folder, 'widget.html')

@app.route('/register_service')
def register_service():
    """Provide extension metadata to BlueOS"""
    return jsonify({
        "name": "Airmar 300WX",
        "description": "Airmar 300WX WeatherStation interface for BlueOS",
        "icon": "mdi-weather-windy",
        "company": "Blue Robotics",
        "version": "1.0.1",
        "webpage": "https://github.com/vshie/Airmar-WX",
        "api": "https://github.com/vshie/Airmar-WX"
    })

@app.route('/docs')
def docs():
    """Serve API documentation"""
    return jsonify({
        "openapi": "3.0.0",
        "info": {
            "title": "Airmar 300WX API",
            "version": "1.0.1",
            "description": "API for Airmar 300WX WeatherStation"
        },
        "paths": {
            "/api/serial/ports": {
                "get": {
                    "summary": "Get available serial ports",
                    "responses": {
                        "200": {
                            "description": "List of available ports"
                        }
                    }
                }
            }
        }
    })

@app.route('/v1.0/ui/')
def ui():
    """Serve the UI"""
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/api/serial/ports', methods=['GET'])
def get_ports():
    """Get list of available serial ports"""
    return jsonify({"ports": nmea_handler.get_ports()})

@app.route('/api/serial/device-ids', methods=['GET'])
def get_device_ids():
    """Get device ID mappings from /dev/serial/by-id/"""
    return jsonify({"devices": nmea_handler.get_device_ids()})

@app.route('/api/serial/select', methods=['POST'])
def select_port():
    """Select and connect to a serial port"""
    data = request.get_json()
    if not data or 'port' not in data:
        return jsonify({"success": False, "message": "No port specified"})
    
    # Use saved baud as a hint for which rate to try first (e.g. 115200 if already switched)
    if data['port'] == nmea_handler.state.get('port'):
        baud_rate = nmea_handler.state.get('baud_rate', 4800)
    else:
        baud_rate = 4800
    stay_at_4800 = bool(data.get('stay_at_4800', False))
    success, message = nmea_handler.connect_serial(data['port'], baud_rate, stay_at_4800=stay_at_4800)
    return jsonify({"success": success, "message": message})

@app.route('/api/serial/cancel', methods=['POST'])
def cancel_connect():
    """Cancel any in-progress connection attempt"""
    nmea_handler._cancel_connect = True
    nmea_handler._set_conn_status(
        nmea_handler.CONN_STATUS_DISCONNECTED, 'Cancelling...')
    return jsonify({"success": True})

@app.route('/api/serial/disconnect', methods=['POST'])
def disconnect_port():
    """Disconnect from serial port"""
    success, message = nmea_handler.disconnect_serial()
    return jsonify({"success": success, "message": message})

@app.route('/api/serial', methods=['GET'])
def get_serial_info():
    """Get current serial port information"""
    if nmea_handler.serial_connection and nmea_handler.serial_connection.is_open:
        return jsonify({
            "serial_port": nmea_handler.serial_connection.port,
            "baud_rate": nmea_handler.serial_connection.baudrate,
            "detected_baud": nmea_handler.detected_baud
        })
    return jsonify({"serial_port": "Not connected", "baud_rate": 0, "detected_baud": None})

@app.route('/api/connection/status', methods=['GET'])
def get_connection_status():
    """Get detailed connection status information"""
    return jsonify(nmea_handler.get_connection_info())

@app.route('/api/sensor/state', methods=['GET'])
def get_sensor_state():
    """Get aggregated sensor data for dashboard display"""
    return jsonify(nmea_handler.get_sensor_data())

@app.route('/api/sensor/history', methods=['GET'])
def get_sensor_history():
    """Get historical sensor data for sparklines (15 min).

    Optional query param ``keys``: comma-separated series names to include
    (e.g. ``?keys=wind_apparent_paired,wind_true_paired``). When omitted,
    all series are returned. ``history_window_seconds`` is always included.
    """
    keys_param = request.args.get('keys', '').strip()
    keys = [k.strip() for k in keys_param.split(',') if k.strip()] if keys_param else None
    return jsonify(nmea_handler.get_sensor_history(keys=keys))


@app.route('/api/sensor/history/reset-wind-paired', methods=['POST'])
def reset_wind_paired_history():
    """Clear apparent or true paired wind history (roses and heatmaps for that stream)."""
    data = request.get_json() or {}
    which = data.get('which', '')
    if which not in ('apparent', 'true'):
        return jsonify({'error': 'which must be "apparent" or "true"'}), 400
    nmea_handler.reset_wind_paired_history(which)
    return jsonify({'ok': True})

# ============== Sentence Configuration API ==============

@app.route('/api/sentences', methods=['GET'])
def get_sentences():
    """Get list of all supported NMEA sentences with their info and saved config (persisted across restarts)."""
    return jsonify({
        "sentences": nmea_handler.get_sentences_info(),
        "connected": nmea_handler.serial_connection is not None and nmea_handler.serial_connection.is_open,
        "saved_config": nmea_handler.state.get('sentence_config') or {}
    })

@app.route('/api/sentences/configure', methods=['POST'])
def configure_sentence():
    """Configure a single NMEA sentence (enable/disable, set interval).
    interval: seconds (0.1–5), converted to device tenths-of-seconds internally."""
    data = request.get_json()
    if not data or 'sentence_id' not in data:
        return jsonify({"success": False, "message": "No sentence_id specified"})
    
    sentence_id = data['sentence_id']
    enabled = data.get('enabled', True)
    interval_raw = data.get('interval', None)
    # Convert seconds (0.1–5) to tenths (1–50) if provided
    interval = None
    if interval_raw is not None:
        try:
            sec = float(interval_raw)
            sec = max(0.1, min(5.0, sec))
            interval = int(round(sec * 10))
        except (TypeError, ValueError):
            pass
    
    success, message = nmea_handler.configure_sentence(sentence_id, enabled, interval)
    return jsonify({"success": success, "message": message})

@app.route('/api/sentences/configure-batch', methods=['POST'])
def configure_sentences_batch():
    """Configure multiple NMEA sentences in one call.
    Body: { changes: [{sentence_id, enabled, interval(seconds 0.1–5)}] }"""
    data = request.get_json() or {}
    changes_in = data.get('changes', [])
    if not isinstance(changes_in, list) or not changes_in:
        return jsonify({"success": False, "message": "No changes provided"})
    # Convert interval seconds -> tenths for device
    changes = []
    for ch in changes_in:
        if not isinstance(ch, dict):
            continue
        sentence_id = ch.get('sentence_id')
        enabled = ch.get('enabled', True)
        interval_raw = ch.get('interval', None)
        interval = None
        if interval_raw is not None:
            try:
                sec = float(interval_raw)
                sec = max(0.1, min(5.0, sec))
                interval = int(round(sec * 10))
            except (TypeError, ValueError):
                interval = None
        changes.append({"sentence_id": sentence_id, "enabled": enabled, "interval": interval})
    success, message = nmea_handler.configure_sentences_batch(changes)
    return jsonify({"success": success, "message": message})

@app.route('/api/sentences/query', methods=['POST'])
def query_sentences():
    """Query the device for current sentence configuration"""
    success, result = nmea_handler.query_sentence_config()
    if success:
        return jsonify({"success": True, "config": result})
    else:
        return jsonify({"success": False, "message": result})

@app.route('/api/sentences/save', methods=['POST'])
def save_sentences():
    """Save current sentence configuration to device EEPROM"""
    success, message = nmea_handler.save_sentence_config()
    return jsonify({"success": success, "message": message})

@app.route('/api/sentences/load-defaults', methods=['POST'])
def load_sentence_defaults():
    """Load factory default sentence configuration"""
    success, message = nmea_handler.load_sentence_defaults()
    return jsonify({"success": success, "message": message})

# ============== End Sentence Configuration API ==============

@app.route('/api/read', methods=['GET'])
def read_serial():
    """Read data from serial port"""
    return jsonify(nmea_handler.read_serial())

@app.route('/api/log_message', methods=['POST'])
def log_message():
    """Log NMEA message"""
    data = request.get_json()
    if not data or 'message' not in data:
        return jsonify({"success": False, "message": "No message specified"})
    
    success, message = nmea_handler.log_message(data['message'])
    return jsonify({"success": success, "message": message})

@app.route('/api/logs', methods=['GET'])
def download_logs():
    """Download log file"""
    if not nmea_handler.log_path.exists():
        return jsonify({
            "success": False,
            "message": "No log file found. Connect to a device and receive messages to create logs."
        }), 404
    return send_file(nmea_handler.log_path, as_attachment=True)

@app.route('/api/logs/delete', methods=['POST'])
def delete_logs():
    """Delete log file"""
    try:
        if nmea_handler.log_path.exists():
            nmea_handler.log_path.unlink()
        return jsonify({"success": True, "message": "Logs deleted successfully"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})

@app.route('/api/logs/app/delete', methods=['POST'])
def delete_app_logs():
    """Delete application log file"""
    try:
        app_log_path = Path('/app/logs/300wx.log')
        if app_log_path.exists():
            app_log_path.unlink()
        return jsonify({"success": True, "message": "Application log deleted successfully"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})

@app.route('/api/logs/app', methods=['GET'])
def download_app_logs():
    """Download application log file"""
    app_log_path = Path('/app/logs/300wx.log')
    if not app_log_path.exists():
        return jsonify({
            "success": False,
            "message": "No application log file found."
        }), 404
    return send_file(app_log_path, as_attachment=True)

@app.route('/api/logs/info', methods=['GET'])
def get_logs_info():
    """Get information about log files"""
    log_dir = Path('/app/logs')
    logs = []
    
    for log_file in [nmea_handler.log_path, log_dir / '300wx.log']:
        if log_file.exists():
            stat = log_file.stat()
            logs.append({
                'name': log_file.name,
                'path': str(log_file),
                'size_bytes': stat.st_size,
                'size_human': _format_size(stat.st_size),
                'modified': datetime.datetime.fromtimestamp(stat.st_mtime).isoformat(),
                'modified_human': _format_time_ago(stat.st_mtime)
            })
        else:
            logs.append({
                'name': log_file.name,
                'path': str(log_file),
                'size_bytes': 0,
                'size_human': '0 B',
                'modified': None,
                'modified_human': 'Not created'
            })
    
    return jsonify({
        'log_directory': str(log_dir),
        'logs': logs
    })

@app.route('/api/logs/preview', methods=['GET'])
def get_log_preview():
    """Get the last N lines of a log file"""
    log_type = request.args.get('type', 'nmea')  # 'nmea' or 'app'
    lines = int(request.args.get('lines', 50))
    
    if log_type == 'nmea':
        log_path = nmea_handler.log_path
    else:
        log_path = Path('/app/logs/300wx.log')
    
    if not log_path.exists():
        return jsonify({'lines': [], 'total_lines': 0})
    
    try:
        with open(log_path, 'r') as f:
            all_lines = f.readlines()
            total = len(all_lines)
            preview_lines = all_lines[-lines:] if total > lines else all_lines
            return jsonify({
                'lines': [line.rstrip() for line in preview_lines],
                'total_lines': total,
                'showing': len(preview_lines)
            })
    except Exception as e:
        return jsonify({'error': str(e), 'lines': [], 'total_lines': 0})

def _format_size(size_bytes):
    """Format bytes to human readable size"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size_bytes < 1024:
            return f"{size_bytes:.1f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.1f} TB"

def _format_time_ago(timestamp):
    """Format timestamp to human readable time ago"""
    now = datetime.datetime.now()
    dt = datetime.datetime.fromtimestamp(timestamp)
    diff = now - dt
    
    if diff.days > 0:
        return f"{diff.days}d ago"
    elif diff.seconds >= 3600:
        return f"{diff.seconds // 3600}h ago"
    elif diff.seconds >= 60:
        return f"{diff.seconds // 60}m ago"
    else:
        return "Just now"

@app.route('/api/stream/start', methods=['POST'])
def start_streaming():
    """Start UDP streaming"""
    success, message = nmea_handler.start_streaming()
    return jsonify({"success": success, "message": message})

@app.route('/api/stream/stop', methods=['POST'])
def stop_streaming():
    """Stop UDP streaming"""
    success, message = nmea_handler.stop_streaming()
    return jsonify({"success": success, "message": message})

@app.route('/api/stream/status', methods=['GET'])
def get_streaming_status():
    """Get current dual-route streaming status.

    The stream now runs both routes (wind → 27001, GPS → 27002) at the same
    time; the payload therefore reports a `routes` array with per-port
    counters plus the persisted ArduRover setup snapshot.
    """
    snap = nmea_handler._stream_status_snapshot()
    snap["port"] = nmea_handler.state.get('port')
    snap["baud_rate"] = nmea_handler.state.get('baud_rate')
    return jsonify(snap)

@app.route('/api/autopilot/setup', methods=['GET'])
def get_autopilot_setup():
    """Return persisted ArduRover setup snapshot (SERIAL X/Y, flags, expected)."""
    return jsonify({
        "success": True,
        "status": nmea_handler.get_autopilot_setup_status(),
    })

@app.route('/api/autopilot/setup', methods=['POST'])
def post_autopilot_setup():
    """Apply the ArduRover parameter setup for the chosen SERIAL X/Y.

    Body: { "wind_serial": int, "gps_serial": int,
            "use_gps_yaw_fallback": bool (optional, default false) }
    """
    data = request.get_json(silent=True) or {}
    wind_serial = data.get('wind_serial')
    gps_serial = data.get('gps_serial')
    yaw = bool(data.get('use_gps_yaw_fallback', False))
    ok, message, payload = nmea_handler.apply_autopilot_setup(
        wind_serial, gps_serial, yaw,
    )
    return jsonify({"success": ok, "message": message, **payload})

@app.route('/api/autopilot/check', methods=['GET', 'POST'])
def check_autopilot():
    """Report parameter drift vs the persisted expected snapshot."""
    ok, message, payload = nmea_handler.check_autopilot_setup()
    return jsonify({"success": ok, "message": message, **payload})

@app.route('/api/autopilot/restore', methods=['POST'])
def restore_autopilot():
    """Re-apply the persisted expected snapshot (Restore button)."""
    ok, message, payload = nmea_handler.restore_autopilot_setup()
    return jsonify({"success": ok, "message": message, **payload})

@app.route('/api/autopilot/ignore_drift', methods=['POST'])
def ignore_autopilot_drift():
    """Silence the drift banner permanently for the current SERIAL X/Y pair."""
    ok, message, payload = nmea_handler.ignore_autopilot_drift()
    return jsonify({"success": ok, "message": message, **payload})

@app.route('/api/mavlink/nvf_status', methods=['GET'])
def mavlink_nvf_status():
    """Diagnostics for the NAMED_VALUE_FLOAT publisher.

    Useful when the user wants to confirm WX_AppSpd / WX_AppDir / WX_TruSpd /
    WX_TruDir are being POSTed to mavlink2rest and accepted (which is what
    makes them appear in the autopilot's .BIN log as `NVF` rows, alongside
    ArduPilot's own AppWndSpd / AppWndDir wind-vane emissions).
    """
    return jsonify(nmea_handler.get_mavlink_nvf_status())

@app.route('/api/serial/change_baud', methods=['POST'])
def change_baud():
    """Change the baud rate of the weather station"""
    data = request.get_json()
    if not data or 'baud_rate' not in data:
        return jsonify({"success": False, "message": "No baud rate specified"})
    
    baud_rate = data['baud_rate']
    allowed = [4800, nmea_handler.OPERATING_BAUD_RATE]
    if baud_rate not in allowed:
        return jsonify({
            "success": False,
            "message": f"Invalid baud rate. Must be one of: {allowed}",
        })
    
    success, message = nmea_handler.change_baud_rate(baud_rate)
    return jsonify({"success": success, "message": message})

if __name__ == '__main__':
    from waitress import serve
    # SSE clients hold a worker thread each; >4 default threads avoids queue backlog.
    serve(app, host='0.0.0.0', port=6436, threads=16)
