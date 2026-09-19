"""
ArduRover parameter check / apply for the Airmar-WX extension.

What this configures
--------------------
After the user picks two distinct SERIALx indexes -- one for the wind
(`udpin:0.0.0.0:27001`, drives AP_WindVane_NMEA) and one for GPS/heading
(`udpin:0.0.0.0:27002`, drives AP_GPS NMEA + HDT yaw) -- this module writes
the ArduRover parameters that make each driver actually consume the stream:

Wind serial X:
    SERIAL{X}_PROTOCOL = 21  (WindVane)
    WNDVN_TYPE         = 4   (NMEA)
    WNDVN_SPEED_TYPE   = 4   (NMEA; without this, speed defaults to none)

GPS serial Y:
    SERIAL{Y}_PROTOCOL = 5   (GPS)
    GPS1_TYPE          = 5   (NMEA) — falls back to legacy GPS_TYPE if the
                                     firmware still ships the pre-multi-GPS name

Secondary EKF source set (default): use the Airmar GPS + HDT as a complete
alternate source that a Lua script or aux switch can promote later. Not
touched at runtime by ArduPilot until the user selects source set 2 via
`MAV_CMD_SET_EKF_SOURCE_SET` or `RCx_OPTION=90`. Default
`EK3_SRC2_POSXY`/`_VELXY` is None, so writing them here is required or the
alternate set would drop horizontal position:

    EK3_SRC2_YAW    = 2  (GPS)
    EK3_SRC2_POSXY  = 3  (GPS)
    EK3_SRC2_VELXY  = 3  (GPS)

Optional user opt-in (`use_gps_yaw_fallback = True`): promote GPS-reported
yaw to the *active* source set with compass fallback. The Airmar HDT is a
magnetometer-derived heading (not dual-antenna GPS yaw), so this is only
appropriate when the on-board compass is worse than the Airmar heading:

    EK3_SRC1_YAW = 3  (GPS with compass fallback)

We deliberately leave SRC1_POSXY/VELXY/POSZ alone — the primary set already
uses the GPS driver for position on the Rover default profile.

What this module does NOT touch:
* `SERIALn_BAUD`  — UDPIN sockets ignore serial baud and forcing a value would
  clobber a real UART if the user reuses the same index for wired hardware.
* Any serial index that is not X or Y.
* `AHRS_EKF_TYPE`, `EK3_ENABLE`, `EK3_MAG_CAL`, or any compass config.
* The BlueOS Autopilot Firmware serial-device string
  (`udpin:0.0.0.0:2700X`). That mapping is stored in BlueOS, not in
  ArduPilot parameters, and has no MAVLink write path — the user still
  pastes the two strings into the serial configuration UI.

Apply-once model
----------------
Writing is triggered only when the user submits the setup form. After a
successful apply, the expected values are persisted; a background/UI check
compares them to current values and, on drift, offers Restore or Ignore.
Ignore is sticky (never nag again) until the user changes X, Y, or the yaw
fallback checkbox — those form a new setup.

MAVLink transport
-----------------
Uses BlueOS mavlink2rest over HTTP (no pymavlink). Endpoint list mirrors
`mavlink_sender.py` so both modules share the same fallback ladder. A
PARAM_SET is considered successful only after a fresh PARAM_VALUE echo with
a matching name arrives after the request timestamp — this avoids reading a
stale cached mailbox entry and reporting a phantom success.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import requests

log = logging.getLogger('app')


# ── HTTP endpoints (parallels mavlink_sender.POST_ENDPOINTS) ─────────
# mavlink2rest has moved paths across BlueOS releases. Try the reverse-proxy
# path first (works on stock BlueOS) and fall back to the direct port.
_MAVLINK_POST_ENDPOINTS = (
    'http://host.docker.internal/mavlink2rest/mavlink',
    'http://host.docker.internal:6040/v1/mavlink',
    'http://192.168.2.2/mavlink2rest/mavlink',
    'http://192.168.2.2:6040/v1/mavlink',
    'http://localhost/mavlink2rest/mavlink',
    'http://localhost:6040/v1/mavlink',
    'http://blueos.local/mavlink2rest/mavlink',
    'http://blueos.local:6040/v1/mavlink',
)

# The GET side of mavlink2rest is derived from the POST endpoint (strip the
# `/mavlink` tail). We fetch messages by name at `<base>/mavlink/<msg>`.
def _get_base_for(post_endpoint: str) -> str:
    if post_endpoint.endswith('/mavlink'):
        return post_endpoint[: -len('/mavlink')]
    return post_endpoint


# ── MAVLink identifiers ──────────────────────────────────────────────
# GCS-style sender, same as the NVF publisher so the autopilot bookkeeps
# both streams under one system id and different component ids.
_GCS_SYSTEM_ID = 255
_GCS_COMPONENT_ID = 240  # MAV_COMP_ID_ONBOARD_COMPUTER-ish, matches SubReels

# Target the local ArduPilot; on BlueOS this is (1, 1) for the flight
# controller. We do not currently support boat-boat setups from this
# extension because there is no consumer for it yet.
_AUTOPILOT_SYSTEM_ID = 1
_AUTOPILOT_COMPONENT_ID = 1

_PARAM_TYPE_REAL32 = 9  # MAV_PARAM_TYPE_REAL32; ArduPilot ignores this hint
                        # and stores using its own on-disk type, so REAL32 is
                        # safe for every ArduPilot param.


# ── Setup contract ───────────────────────────────────────────────────
# Valid SERIALx range on ArduPilot builds we care about. SERIAL0 is the
# USB console; refusing it prevents accidentally breaking the GCS link.
MIN_SERIAL_INDEX = 1
MAX_SERIAL_INDEX = 9

# Serial protocol enum values (see AP_SerialManager::SerialProtocol).
SERIAL_PROTOCOL_GPS = 5
SERIAL_PROTOCOL_WINDVANE = 21

# NMEA driver "type" for the WindVane and GPS libraries.
WNDVN_TYPE_NMEA = 4
GPS_TYPE_NMEA = 5

# EKF3 source enums (see AP_NavEKF_Source).
EK3_YAW_GPS = 2
EK3_YAW_GPS_WITH_COMPASS_FALLBACK = 3
EK3_POSXY_GPS = 3
EK3_VELXY_GPS = 3


# ── Public helpers ───────────────────────────────────────────────────

def validate_selection(wind_serial: Optional[int],
                       gps_serial: Optional[int]) -> Tuple[bool, str]:
    """Guard: both must be integers in range and distinct."""
    for role, value in (('wind_serial', wind_serial), ('gps_serial', gps_serial)):
        if not isinstance(value, int) or isinstance(value, bool):
            return False, f"{role} must be an integer SERIAL index"
        if value < MIN_SERIAL_INDEX or value > MAX_SERIAL_INDEX:
            return False, (
                f"{role} must be between SERIAL{MIN_SERIAL_INDEX} "
                f"and SERIAL{MAX_SERIAL_INDEX} (got {value})"
            )
    if wind_serial == gps_serial:
        return False, "wind_serial and gps_serial must be different SERIAL ports"
    return True, ''


def build_expected_params(wind_serial: int,
                          gps_serial: int,
                          use_gps_yaw_fallback: bool = False) -> Dict[str, float]:
    """Return the {param_name: value} we want ArduPilot to end up with.

    Note: `GPS1_TYPE` is the modern name; ArduPilot still accepts `GPS_TYPE`
    on older firmwares. The apply path probes both and writes whichever
    exists; both are listed here so the check step reports whichever the
    autopilot actually has.
    """
    exp: Dict[str, float] = {
        # Wind serial X
        f'SERIAL{wind_serial}_PROTOCOL': float(SERIAL_PROTOCOL_WINDVANE),
        'WNDVN_TYPE': float(WNDVN_TYPE_NMEA),
        'WNDVN_SPEED_TYPE': float(WNDVN_TYPE_NMEA),
        # GPS serial Y
        f'SERIAL{gps_serial}_PROTOCOL': float(SERIAL_PROTOCOL_GPS),
        'GPS1_TYPE': float(GPS_TYPE_NMEA),
        # Secondary EKF source set — full alternate GPS-yaw set
        'EK3_SRC2_YAW': float(EK3_YAW_GPS),
        'EK3_SRC2_POSXY': float(EK3_POSXY_GPS),
        'EK3_SRC2_VELXY': float(EK3_VELXY_GPS),
    }
    if use_gps_yaw_fallback:
        exp['EK3_SRC1_YAW'] = float(EK3_YAW_GPS_WITH_COMPASS_FALLBACK)
    return exp


def _param_synonyms(name: str) -> List[str]:
    """Return acceptable aliases for a canonical param name.

    Currently only `GPS1_TYPE` has a legacy alias (`GPS_TYPE`, pre-multi-GPS
    firmwares). Return the canonical name first so callers write to the
    modern name when both exist.
    """
    if name == 'GPS1_TYPE':
        return ['GPS1_TYPE', 'GPS_TYPE']
    return [name]


def _values_match(a: float, b: float) -> bool:
    """Loose equality tolerant of floating-point round-trips through JSON."""
    try:
        return abs(float(a) - float(b)) <= max(1e-4, abs(float(b)) * 1e-3)
    except (TypeError, ValueError):
        return False


# ── ParamClient ──────────────────────────────────────────────────────

class ParamClient:
    """Thread-safe mavlink2rest parameter client.

    One instance per NMEAHandler; use the same instance across requests so
    the endpoint cache and per-transaction lock survive.
    """

    def __init__(self,
                 endpoints=_MAVLINK_POST_ENDPOINTS,
                 timeout_s: float = 3.0):
        self._endpoints = tuple(endpoints)
        self._timeout_s = timeout_s
        self._session = requests.Session()
        self._cached_endpoint: Optional[str] = None
        # Serialize parameter transactions: mavlink2rest keeps a single
        # PARAM_VALUE mailbox per (sys,comp) and interleaved reads would
        # race for it.
        self._lock = threading.Lock()

    # -- endpoint selection ------------------------------------------------

    def _candidates(self) -> List[str]:
        cached = self._cached_endpoint
        if cached is None:
            return list(self._endpoints)
        return [cached] + [e for e in self._endpoints if e != cached]

    def _post(self, payload: dict) -> Tuple[bool, Optional[str]]:
        """POST `payload` to the first working endpoint. Cache it."""
        last_error: Optional[str] = None
        for endpoint in self._candidates():
            try:
                r = self._session.post(endpoint, json=payload, timeout=self._timeout_s)
            except Exception as e:  # network unreachable / DNS / etc
                last_error = f"{endpoint}: {e}"
                continue
            if 200 <= r.status_code < 300:
                if self._cached_endpoint != endpoint:
                    log.info("mavlink2rest endpoint (param client): %s", endpoint)
                    self._cached_endpoint = endpoint
                return True, None
            last_error = f"{endpoint}: HTTP {r.status_code} {r.text[:120]!s}"
            # Non-2xx invalidates the cache so the next attempt re-probes.
            if self._cached_endpoint == endpoint:
                self._cached_endpoint = None
        return False, last_error

    def _get_param_value(self) -> Optional[dict]:
        """GET the last PARAM_VALUE message stored by mavlink2rest.

        Returns the decoded JSON (structure varies slightly between
        mavlink2rest versions) or None. The caller is expected to sanity
        check the `param_id` field before trusting the value.
        """
        last_error: Optional[str] = None
        for endpoint in self._candidates():
            base = _get_base_for(endpoint)
            # Prefer the message-by-name shorthand; fall back to the
            # component-scoped path some versions expose.
            for url in (
                f"{base}/mavlink/PARAM_VALUE",
                f"{base}/mavlink/vehicles/{_AUTOPILOT_SYSTEM_ID}/components/{_AUTOPILOT_COMPONENT_ID}/messages/PARAM_VALUE",
            ):
                try:
                    r = self._session.get(url, timeout=self._timeout_s)
                except Exception as e:
                    last_error = f"{url}: {e}"
                    continue
                if 200 <= r.status_code < 300:
                    try:
                        return r.json()
                    except Exception as e:
                        last_error = f"{url}: json decode: {e}"
                        continue
                # 404 is normal until the first PARAM_VALUE ever arrives.
                last_error = f"{url}: HTTP {r.status_code}"
        if last_error:
            log.debug("PARAM_VALUE GET fell through: %s", last_error)
        return None

    # -- payload builders --------------------------------------------------

    @staticmethod
    def _param_id_field(name: str) -> List[str]:
        """Encode PARAM_SET/REQUEST_READ id as 16 one-char strings, null-padded."""
        out: List[str] = []
        for i in range(16):
            out.append(name[i] if i < len(name) else '\x00')
        return out

    def _param_request_read_payload(self, name: str) -> dict:
        return {
            'header': {
                'system_id': _GCS_SYSTEM_ID,
                'component_id': _GCS_COMPONENT_ID,
                'sequence': 0,
            },
            'message': {
                'type': 'PARAM_REQUEST_READ',
                'target_system': _AUTOPILOT_SYSTEM_ID,
                'target_component': _AUTOPILOT_COMPONENT_ID,
                'param_id': self._param_id_field(name),
                'param_index': -1,  # -1 means "lookup by name"
            },
        }

    def _param_set_payload(self, name: str, value: float) -> dict:
        return {
            'header': {
                'system_id': _GCS_SYSTEM_ID,
                'component_id': _GCS_COMPONENT_ID,
                'sequence': 0,
            },
            'message': {
                'type': 'PARAM_SET',
                'target_system': _AUTOPILOT_SYSTEM_ID,
                'target_component': _AUTOPILOT_COMPONENT_ID,
                'param_id': self._param_id_field(name),
                'param_value': float(value),
                'param_type': {'type': 'MAV_PARAM_TYPE_REAL32'},
            },
        }

    # -- PARAM_VALUE parsing ----------------------------------------------

    @staticmethod
    def _extract_param_value(msg: Optional[dict]) -> Optional[Tuple[str, float]]:
        """Pull (param_id, param_value) out of a PARAM_VALUE JSON blob.

        mavlink2rest wraps messages differently across versions; we accept
        either the naked `{message: {...}}` shape or a `{status:{time:{...}},
        message:{...}}` wrapper.
        """
        if not isinstance(msg, dict):
            return None
        message = msg.get('message', msg)
        if not isinstance(message, dict):
            return None
        if message.get('type') not in (None, 'PARAM_VALUE'):
            return None
        pid = message.get('param_id')
        if isinstance(pid, list):
            name = ''.join(c for c in pid if isinstance(c, str) and c != '\x00').strip()
        elif isinstance(pid, str):
            name = pid.replace('\x00', '').strip()
        else:
            return None
        if not name:
            return None
        value = message.get('param_value')
        if value is None:
            return None
        try:
            return name, float(value)
        except (TypeError, ValueError):
            return None

    # -- public API --------------------------------------------------------

    def read(self, name: str, timeout_s: float = 3.0) -> Optional[float]:
        """Return current value of `name`, or None on timeout / missing param.

        Sends PARAM_REQUEST_READ and polls PARAM_VALUE until a matching
        `param_id` arrives. Missing params (e.g. `WNDVN_*` on a build
        without wind vane) simply time out — we never fabricate a value.
        """
        with self._lock:
            ok, err = self._post(self._param_request_read_payload(name))
            if not ok:
                log.debug("PARAM_REQUEST_READ %s failed: %s", name, err)
                return None
            deadline = time.monotonic() + timeout_s
            poll_interval = 0.15
            re_request_at = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                got = self._get_param_value()
                pair = self._extract_param_value(got)
                if pair is not None and pair[0] == name:
                    return pair[1]
                # If the mailbox hasn't updated in a second, re-ask. Some
                # mavlink2rest builds occasionally drop the first request.
                if time.monotonic() >= re_request_at:
                    self._post(self._param_request_read_payload(name))
                    re_request_at = time.monotonic() + 1.0
                time.sleep(poll_interval)
        return None

    def write(self, name: str, value: float, timeout_s: float = 5.0) -> Tuple[bool, str]:
        """Write `value` to `name` and confirm via a fresh PARAM_VALUE echo.

        We snapshot the current PARAM_VALUE (name+value) before POSTing so
        we can detect a stale mailbox hit. Success requires seeing a
        PARAM_VALUE whose name matches AND whose value is close to what we
        just wrote, arriving strictly after the POST.
        """
        with self._lock:
            request_ts = time.monotonic()
            ok, err = self._post(self._param_set_payload(name, value))
            if not ok:
                return False, f"POST failed: {err or 'unknown error'}"
            deadline = request_ts + timeout_s
            poll_interval = 0.15
            re_request_at = request_ts + 1.5
            last_seen: Optional[Tuple[str, float]] = None
            while time.monotonic() < deadline:
                got = self._get_param_value()
                pair = self._extract_param_value(got)
                if pair is not None:
                    last_seen = pair
                    if pair[0] == name and _values_match(pair[1], value):
                        return True, 'ok'
                if time.monotonic() >= re_request_at:
                    # Nudge the autopilot to re-emit PARAM_VALUE for `name`
                    # so we do not stall waiting for a broadcast update.
                    self._post(self._param_request_read_payload(name))
                    re_request_at = time.monotonic() + 1.5
                time.sleep(poll_interval)
        if last_seen is None:
            return False, 'timed out waiting for PARAM_VALUE'
        return False, (
            f"timed out; last PARAM_VALUE was {last_seen[0]}={last_seen[1]!r}"
        )

    # -- higher-level operations ------------------------------------------

    def read_expected(self, expected: Dict[str, float]) -> Dict[str, dict]:
        """Read every expected param, resolving synonyms.

        Returns { canonical_name: {
            'target': float, 'current': float|None, 'match': bool,
            'resolved_name': str|None, 'available': bool, 'reason': str
        }}.
        """
        out: Dict[str, dict] = {}
        for name, target in expected.items():
            row = {
                'target': float(target),
                'current': None,
                'match': False,
                'resolved_name': None,
                'available': False,
                'reason': '',
            }
            for alias in _param_synonyms(name):
                current = self.read(alias)
                if current is not None:
                    row['current'] = current
                    row['resolved_name'] = alias
                    row['available'] = True
                    row['match'] = _values_match(current, target)
                    break
            if not row['available']:
                row['reason'] = 'parameter not found on autopilot'
            out[name] = row
        return out

    def apply_expected(self, expected: Dict[str, float]) -> Dict[str, dict]:
        """Write every expected param, skipping ones that already match.

        Returns { canonical_name: {
            'target', 'previous', 'current', 'action', 'ok', 'reason',
            'resolved_name', 'available'
        }} where `action` is one of 'noop', 'wrote', 'skipped', 'failed'.
        """
        out: Dict[str, dict] = {}
        for name, target in expected.items():
            row = {
                'target': float(target),
                'previous': None,
                'current': None,
                'action': 'skipped',
                'ok': False,
                'reason': '',
                'resolved_name': None,
                'available': False,
            }
            resolved: Optional[str] = None
            for alias in _param_synonyms(name):
                previous = self.read(alias)
                if previous is not None:
                    resolved = alias
                    row['resolved_name'] = alias
                    row['available'] = True
                    row['previous'] = previous
                    if _values_match(previous, target):
                        row['action'] = 'noop'
                        row['current'] = previous
                        row['ok'] = True
                    break
            if not row['available']:
                row['reason'] = 'parameter not found on autopilot; skipped'
                out[name] = row
                continue
            if row['ok']:
                out[name] = row
                continue
            ok, why = self.write(resolved, target)
            if ok:
                row['action'] = 'wrote'
                row['ok'] = True
                row['current'] = float(target)
            else:
                row['action'] = 'failed'
                row['ok'] = False
                row['reason'] = why
            out[name] = row
        return out


# ── State helpers (pure functions; safe to unit-test) ────────────────

def snapshot_from_apply_result(result: Dict[str, dict]) -> Dict[str, float]:
    """Build the persisted `expected` snapshot from an apply result.

    Only params the autopilot actually has get into the snapshot — missing
    ones would otherwise cause the drift check to warn on every poll.
    """
    snap: Dict[str, float] = {}
    for name, row in (result or {}).items():
        if not row.get('available'):
            continue
        target = row.get('target')
        if target is None:
            continue
        snap[name] = float(target)
    return snap


def diff_current_vs_expected(current: Dict[str, dict],
                             expected: Dict[str, float]) -> List[dict]:
    """List entries whose current value drifted from the persisted target."""
    out: List[dict] = []
    for name, target in (expected or {}).items():
        row = current.get(name) or {}
        if not row.get('available'):
            # Parameter vanished (firmware downgrade, etc.) — treat as drift
            # so the user sees it, but flag as unavailable in the message.
            out.append({
                'param': name,
                'expected': float(target),
                'current': None,
                'available': False,
            })
            continue
        if not row.get('match'):
            out.append({
                'param': name,
                'expected': float(target),
                'current': row.get('current'),
                'resolved_name': row.get('resolved_name'),
                'available': True,
            })
    return out


__all__ = [
    'ParamClient',
    'MIN_SERIAL_INDEX',
    'MAX_SERIAL_INDEX',
    'SERIAL_PROTOCOL_GPS',
    'SERIAL_PROTOCOL_WINDVANE',
    'WNDVN_TYPE_NMEA',
    'GPS_TYPE_NMEA',
    'EK3_YAW_GPS',
    'EK3_YAW_GPS_WITH_COMPASS_FALLBACK',
    'EK3_POSXY_GPS',
    'EK3_VELXY_GPS',
    'validate_selection',
    'build_expected_params',
    'snapshot_from_apply_result',
    'diff_current_vs_expected',
]
