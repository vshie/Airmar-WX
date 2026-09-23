"""
ArduRover parameter check / apply for the Airmar-WX extension.

The transport layer is a straight port of the pattern proven live on
`vshie/SubReels_TowFish` (see its `app/mavlink_params.py`). Three details
matter for reliability and were missing from the earlier implementation:

1. **mavlink2rest returns HTTP 200 for malformed messages**, with the body
   `"Failed to parse message, not a valid MAVLinkMessage."`. The previous
   code treated that as success, so every PARAM_SET was silently dropped by
   mavlink2rest and never reached the autopilot — the UI happily reported
   "wrote (unverified)" for writes that never went out.
2. **Message body must be built from the server's own template**, fetched
   from `/helper/mavlink?name=<TYPE>`. That ensures every required field
   is present with the type wrapping mavlink2rest currently expects; a
   single missing field triggers the "Failed to parse message" response.
3. **PARAM_VALUE freshness is detected via `status.time.last_update`** on
   the mavlink2rest mailbox wrapper, not by comparing to our own clock
   (which is not comparable to the remote's) nor by relying on value
   equality (autopilot may clamp/round the stored value).

What this module configures
---------------------------
After the user picks two distinct SERIALx indexes -- one for the wind
(`udpin:0.0.0.0:27001`, drives AP_WindVane_NMEA) and one for GPS/heading
(`udpin:0.0.0.0:27002`, drives AP_GPS NMEA + HDT yaw) -- this module writes
the ArduRover parameters that make each driver actually consume the stream:

Wind serial X:
    SERIAL{X}_PROTOCOL = 21  (WindVane)
    WNDVN_TYPE         = 4   (NMEA)
    WNDVN_SPEED_TYPE   = 4   (NMEA; without this, speed defaults to none)

GPS serial Y (Airmar → GPS2, leaves onboard u-Blox as GPS1):
    SERIAL{Y}_PROTOCOL = 5   (GPS)
    GPS1_TYPE          = 1   (AUTO) — probes u-Blox on the vehicle's stock
                                     GPS; also falls back to legacy
                                     `GPS_TYPE` on pre-multi-GPS firmwares
    GPS2_TYPE          = 5   (NMEA) — the Airmar's UDP feed

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

Apply-once model
----------------
Writing is triggered only when the user submits the setup form. After a
successful apply, the expected values are persisted; a background/UI check
compares them to current values and, on drift, offers Restore or Ignore.
Ignore is sticky (never nag again) until the user changes X, Y, or the yaw
fallback checkbox — those form a new setup.
"""

from __future__ import annotations

import copy
import json
import logging
import threading
import time
from typing import Dict, List, Optional, Tuple

import requests

log = logging.getLogger('app')


# ── mavlink2rest transport ───────────────────────────────────────────
# BlueOS proxies mavlink2rest at `/mavlink2rest/` on the host. Because the
# container runs with NetworkMode=host, `127.0.0.1` is the host itself and
# is the same address the autopilot sees. Using `host.docker.internal`
# (172.18.0.1, the docker bridge gateway) worked for GET but caused the
# UDP feeds to break -- see the long comment in `main.py:UDP_HOST` -- so
# we standardize the entire extension on localhost. `mavlink_sender.py`
# still tries a broader endpoint list for NAMED_VALUE_FLOAT publishes.
DEFAULT_BASE_URL = "http://127.0.0.1/mavlink2rest"

# 16-byte NUL-padded char array (per MAVLink spec) for PARAM_SET and
# PARAM_REQUEST_READ.
PARAM_ID_LEN = 16

# ArduPilot ignores the declared PARAM_SET type and stores using the
# parameter's real on-disk type, so REAL32 is safe for everything.
DEFAULT_PARAM_TYPE = "MAV_PARAM_TYPE_REAL32"


def _chars_to_str(param_id_chars) -> str:
    """``['S','E','R','I','A','L','7',...,'\\x00',...]`` -> ``'SERIAL7'``.

    Handles the two shapes mavlink2rest may emit: a list of single-char
    strings (Python mavlink2rest and BlueOS's current mavlink-server),
    or a padded plain string. Byte-int lists are handled defensively in
    case the underlying serializer ever changes.
    """
    if isinstance(param_id_chars, str):
        return param_id_chars.rstrip("\x00")
    if not isinstance(param_id_chars, (list, tuple)):
        return str(param_id_chars or "").rstrip("\x00")
    out: List[str] = []
    for c in param_id_chars:
        if isinstance(c, str):
            if not c or c == "\x00":
                break
            out.append(c[0])
        elif isinstance(c, int) and not isinstance(c, bool):
            if c == 0:
                break
            byte = c & 0xFF
            if 0x20 <= byte < 0x7F:
                out.append(chr(byte))
    return "".join(out)


def _str_to_chars(param_id: str, pad_len: int = PARAM_ID_LEN) -> List[str]:
    """``'SERIAL7_PROTOCOL'`` -> a NUL-padded 16-element char array."""
    chars = list(param_id)[:pad_len]
    chars.extend("\x00" for _ in range(pad_len - len(chars)))
    return chars


# ── Autopilot discovery ─────────────────────────────────────────────
#
# The autopilot is NOT reliably at (system 1, component 1). On the
# BlueBoat this extension targets, the flight controller answers at
# **system 2, component 1** (`SYSID_THISMAV = 2`), while system 1 holds
# only BlueOS's own onboard-controller heartbeat. Hardcoding (1, 1) is
# what made every apply silently do nothing: PARAM_SET went to a system
# that no autopilot was listening on, and the PARAM_VALUE mailbox for
# (1, 1) stayed permanently empty so every read timed out.
#
# Verified live against 192.168.1.69:
#   sys=1 comp=191  MAV_AUTOPILOT_INVALID / MAV_TYPE_ONBOARD_CONTROLLER
#   sys=2 comp=1    MAV_AUTOPILOT_ARDUPILOTMEGA / MAV_TYPE_SURFACE_BOAT  <- FC
#
# So we discover the autopilot from `/mavlink/vehicles` the same way the
# SubReels_TowFish extension does, and re-discover if it goes away.

# Heartbeats we treat as "this is an autopilot", as opposed to BlueOS
# companion computers (MAV_AUTOPILOT_INVALID / ONBOARD_CONTROLLER) or
# GCS nodes on system 255.
_REAL_AUTOPILOTS = (
    "MAV_AUTOPILOT_ARDUPILOTMEGA",
    "MAV_AUTOPILOT_PX4",
)

# This extension configures a surface vehicle's wind vane + GPS, so when
# several autopilots are visible on a shared MAVLink network prefer the
# boat/rover over anything submerged or airborne.
BOAT_MAVTYPES = (
    "MAV_TYPE_SURFACE_BOAT",
    "MAV_TYPE_GROUND_ROVER",
    "MAV_TYPE_GROUND",
)


def _heartbeat_enum(message: dict, field: str) -> str:
    """Pull a mavlink2rest enum ``type`` string out of a HEARTBEAT field."""
    value = message.get(field)
    if isinstance(value, dict):
        return str(value.get("type") or "")
    return str(value or "")


def pick_autopilot(vehicles, prefer_mavtypes=None):
    """Return ``(system_id, component_id)`` of a real autopilot, or None.

    ``vehicles`` is the JSON object from ``GET /mavlink/vehicles``.
    Companion computers and GCS nodes are ignored. When more than one
    autopilot is visible, ``prefer_mavtypes`` (e.g. :data:`BOAT_MAVTYPES`)
    picks the one whose HEARTBEAT.mavtype matches, in listed order.
    """
    if not isinstance(vehicles, dict):
        return None
    found = []
    for vid, vehicle in vehicles.items():
        if not isinstance(vehicle, dict):
            continue
        try:
            sysid = int(vid)
        except (TypeError, ValueError):
            continue
        components = vehicle.get("components") or {}
        if not isinstance(components, dict):
            continue
        for cid, component in components.items():
            if not isinstance(component, dict):
                continue
            try:
                comp = int(cid)
            except (TypeError, ValueError):
                continue
            heartbeat = (((component.get("messages") or {}).get("HEARTBEAT")
                          or {}).get("message") or {})
            if not isinstance(heartbeat, dict):
                continue
            if _heartbeat_enum(heartbeat, "autopilot") not in _REAL_AUTOPILOTS:
                continue
            found.append((sysid, comp, _heartbeat_enum(heartbeat, "mavtype")))
    if not found:
        return None
    for preferred in tuple(prefer_mavtypes or ()):
        for sysid, comp, mavtype in found:
            if mavtype == preferred:
                return sysid, comp
    return found[0][0], found[0][1]


# ── Setup contract ──────────────────────────────────────────────────
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
# ArduPilot AP_GPS enum: 0=None, 1=AUTO, 2=uBlox, 5=NMEA (see AP_GPS.h).
# `AUTO` probes u-Blox / SBP / SiRF / ERB but explicitly does NOT probe
# NMEA, so an NMEA receiver must use `5`. The BlueBoat's onboard GPS is a
# u-Blox that AUTO detects; we leave GPS1 on AUTO and put the Airmar NMEA
# feed on GPS2 so both drivers work simultaneously without disturbing the
# vehicle's stock GPS wiring.
GPS_TYPE_AUTO = 1

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

    GPS contract (as of 1.1.6):
    * `GPS1_TYPE = 1` (AUTO) — leave the BlueBoat's onboard u-Blox as
      GPS1. AUTO detects u-Blox / SBP / SiRF / ERB, which covers the
      stock hardware. We do NOT force NMEA on GPS1 because that would
      break the vehicle's built-in receiver.
    * `GPS2_TYPE = 5` (NMEA) — this is the Airmar. The extension only
      writes GPS2; the operator wires the Airmar's UDP stream to the
      autopilot's `SERIAL{gps_serial}` slot via BlueOS.

    Earlier versions wrote `GPS1_TYPE = 5`, which forced the primary GPS
    driver into NMEA mode and disabled the onboard u-Blox. Migration in
    `main.py` upgrades old persisted `autopilot_setup` snapshots.
    """
    exp: Dict[str, float] = {
        # Wind serial X
        f'SERIAL{wind_serial}_PROTOCOL': float(SERIAL_PROTOCOL_WINDVANE),
        'WNDVN_TYPE': float(WNDVN_TYPE_NMEA),
        'WNDVN_SPEED_TYPE': float(WNDVN_TYPE_NMEA),
        # GPS serial Y (Airmar → GPS2)
        f'SERIAL{gps_serial}_PROTOCOL': float(SERIAL_PROTOCOL_GPS),
        'GPS1_TYPE': float(GPS_TYPE_AUTO),
        'GPS2_TYPE': float(GPS_TYPE_NMEA),
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

    Only `GPS1_TYPE` has a legacy alias (`GPS_TYPE`, on pre-multi-GPS
    firmwares that never gained the numbered variant). `GPS2_TYPE` has no
    legacy alias: firmwares old enough to lack numbered GPS params also
    lacked a second GPS instance entirely.
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
    """Read and write autopilot parameters through one mavlink2rest host.

    Port of the ParamClient from SubReels_TowFish, adapted to return
    plain floats / booleans instead of raising. Two callers matter:

    * `apply_expected(expected)` — POST PARAM_SET for each expected param,
      verify via a fresh PARAM_VALUE echo, and return a per-param result
      dict describing the outcome.
    * `read_expected(expected)`  — POST PARAM_REQUEST_READ for each param
      and return the observed value, for the drift-check step.

    Both share the same underlying `read()` / `write()` primitives, which
    serialize access to the single PARAM_VALUE mailbox with a lock.
    """

    def __init__(self,
                 base_url: str = DEFAULT_BASE_URL,
                 target_system: Optional[int] = None,
                 target_component: Optional[int] = None,
                 gcs_system_id: int = 255,
                 gcs_component_id: int = 240,
                 http_timeout_s: float = 2.0) -> None:
        self.base_url = base_url.rstrip("/")
        # `None` means "discover from /mavlink/vehicles on first use".
        # Passing explicit values pins the target (used by tests).
        self.target_system = target_system
        self.target_component = target_component
        self._target_pinned = target_system is not None
        self.gcs_system_id = gcs_system_id
        self.gcs_component_id = gcs_component_id
        self.http_timeout_s = http_timeout_s
        self._session = requests.Session()
        # Per-message-type template cache from /helper/mavlink so we build
        # every payload against the server's own schema.
        self._template_cache: Dict[str, dict] = {}
        # Serialize PARAM_VALUE mailbox access so two concurrent reads
        # can't consume each other's answers.
        self._lock = threading.Lock()
        # One-shot INFO log of the first PARAM_VALUE we see, for
        # debuggability if the shape ever changes again.
        self._logged_first_response = False

    # -- autopilot target ------------------------------------------------

    def discover_target(self) -> Optional[Tuple[int, int]]:
        """Ask mavlink2rest which vehicle/component is the autopilot.

        Returns ``(system_id, component_id)`` or None when the host is
        down or isn't publishing an autopilot HEARTBEAT.
        """
        vehicles = self._get_json("/mavlink/vehicles")
        if vehicles is None:
            return None
        return pick_autopilot(vehicles, prefer_mavtypes=BOAT_MAVTYPES)

    def ensure_target(self, force: bool = False) -> bool:
        """Resolve the autopilot address, caching the result.

        Returns True when we have a usable (system, component). Callers
        must invoke this before any PARAM_* traffic — sending to the
        wrong system is silently ignored by the autopilot and looks
        exactly like "the parameter doesn't exist".
        """
        if self._target_pinned:
            return self.target_system is not None
        if not force and self.target_system is not None:
            return True
        found = self.discover_target()
        if found is None:
            log.warning(
                "No ArduPilot/PX4 autopilot found via %s/mavlink/vehicles; "
                "cannot read or write parameters", self.base_url,
            )
            return False
        if (self.target_system, self.target_component) != found:
            log.info(
                "Autopilot discovered at system %d component %d", *found,
            )
        self.target_system, self.target_component = found
        return True

    # -- low-level HTTP --------------------------------------------------

    def _get_json(self, path: str) -> Optional[dict]:
        url = f"{self.base_url}{path}"
        try:
            r = self._session.get(url, timeout=self.http_timeout_s)
        except Exception as e:
            log.debug("mavlink2rest GET %s failed: %s", url, e)
            return None
        if r.status_code != 200:
            return None
        body = (r.text or "").strip()
        # mavlink2rest answers "None" for a message it has never seen.
        if not body or body == "None":
            return None
        try:
            return json.loads(body)
        except Exception as e:
            log.debug("mavlink2rest GET %s json decode failed: %s", url, e)
            return None

    def _post(self, envelope: dict, info: str) -> bool:
        """POST an already-enveloped message. Returns True on real success.

        mavlink2rest returns HTTP 200 with body `"Failed to parse message,
        not a valid MAVLinkMessage."` on schema mismatch — this is the
        detail the earlier implementation missed. We reject on that body.
        """
        try:
            r = self._session.post(
                f"{self.base_url}/mavlink", json=envelope,
                timeout=self.http_timeout_s,
            )
        except Exception as e:
            log.debug("mavlink2rest POST %s failed: %s", info, e)
            return False
        if r.status_code != 200:
            log.warning(
                "mavlink2rest POST %s -> HTTP %s: %s",
                info, r.status_code, (r.text or "")[:200],
            )
            return False
        body = (r.text or "").strip()
        if body.lower().startswith("failed"):
            log.warning(
                "mavlink2rest rejected %s: %s", info, body[:200],
            )
            return False
        return True

    def _envelope(self, message: dict) -> dict:
        """Wrap a message body in the header mavlink2rest expects.

        Prefers the server's own template for the message type (fetched
        from `/helper/mavlink?name=<TYPE>`) so field names and type
        wrappings track whatever dialect mavlink2rest was built against.
        Falls back to the hand-built body if the helper is unavailable —
        that still works on some builds but is more fragile.
        """
        msg_type = message["type"]
        template = self._template_cache.get(msg_type)
        if template is None:
            fetched = self._get_json(f"/helper/mavlink?name={msg_type}")
            if isinstance(fetched, dict) and "message" in fetched:
                self._template_cache[msg_type] = fetched
                template = fetched
        body = message
        if template is not None:
            body = copy.deepcopy(template["message"])
            body.update(message)
        return {
            "header": {
                "system_id": self.gcs_system_id,
                "component_id": self.gcs_component_id,
                "sequence": 0,
            },
            "message": body,
        }

    # -- PARAM_VALUE mailbox --------------------------------------------

    def _param_value_mailbox(self) -> Tuple[Optional[dict], Optional[str]]:
        """Return ``(message, last_update)`` for the cached PARAM_VALUE.

        ``last_update`` is mavlink2rest's own timestamp string. We only
        ever compare it for equality against a previously observed value,
        never against our own clock (the timestamps come from the
        autopilot host's clock, which is not comparable).
        """
        wrapper = self._get_json(
            f"/mavlink/vehicles/{self.target_system}"
            f"/components/{self.target_component}/messages/PARAM_VALUE"
        )
        if not isinstance(wrapper, dict):
            return None, None
        if not self._logged_first_response:
            self._logged_first_response = True
            log.info(
                "First PARAM_VALUE mailbox payload: %s",
                str(wrapper)[:400],
            )
        message = wrapper.get("message")
        stamp = (((wrapper.get("status") or {}).get("time") or {})
                 .get("last_update"))
        return (message if isinstance(message, dict) else None), stamp

    @staticmethod
    def _decode_param_value(message: dict) -> Tuple[str, Optional[float]]:
        """Return (name, value) from a PARAM_VALUE body."""
        name = _chars_to_str(message.get("param_id"))
        raw = message.get("param_value")
        if isinstance(raw, dict):
            # Some builds wrap primitive fields; unwrap.
            for k in ("value", "val", "data"):
                if k in raw:
                    raw = raw[k]
                    break
        try:
            value = float(raw) if raw is not None else None
        except (TypeError, ValueError):
            value = None
        return name, value

    def _request_read(self, param_id: str) -> bool:
        return self._post(self._envelope({
            "type": "PARAM_REQUEST_READ",
            "target_system": self.target_system,
            "target_component": self.target_component,
            # -1 means "look the parameter up by name, not by index".
            "param_index": -1,
            "param_id": _str_to_chars(param_id),
        }), f"PARAM_REQUEST_READ:{param_id}")

    def _await_param_value(self, param_id: str, deadline: float,
                           reject_stamp: Optional[str],
                           repoke) -> Optional[float]:
        """Poll the PARAM_VALUE mailbox until ``param_id`` shows up.

        ``reject_stamp`` makes the wait ignore a cached message that was
        already there before we asked, which is what turns a write into
        a genuine read-back rather than an echo of the old value.
        ``repoke`` is called ~1s to re-send the request since MAVLink is
        UDP-ish and single requests can be dropped in transit.
        """
        last_poke = time.monotonic()
        while time.monotonic() < deadline:
            message, stamp = self._param_value_mailbox()
            if message is not None:
                name, value = self._decode_param_value(message)
                fresh = reject_stamp is None or stamp != reject_stamp
                if name == param_id and fresh and value is not None:
                    return value
            now = time.monotonic()
            if now - last_poke >= 1.0:
                try:
                    repoke()
                except Exception:
                    pass
                last_poke = now
            time.sleep(0.12)
        return None

    # -- public primitives ---------------------------------------------

    def read(self, param_id: str, timeout_s: float = 3.0) -> Optional[float]:
        """Return the current value of `param_id`, or None on timeout."""
        if not self.ensure_target():
            return None
        with self._lock:
            deadline = time.monotonic() + timeout_s
            if not self._request_read(param_id):
                return None
            value = self._await_param_value(
                param_id, deadline, reject_stamp=None,
                repoke=lambda: self._request_read(param_id),
            )
        if value is None and not self._target_pinned:
            # The autopilot may have rebooted with a different system id,
            # or we cached a stale target. Re-discover once and retry.
            previous = (self.target_system, self.target_component)
            if self.ensure_target(force=True) and \
                    (self.target_system, self.target_component) != previous:
                with self._lock:
                    deadline = time.monotonic() + timeout_s
                    if not self._request_read(param_id):
                        return None
                    return self._await_param_value(
                        param_id, deadline, reject_stamp=None,
                        repoke=lambda: self._request_read(param_id),
                    )
        return value

    def write(self, param_id: str, value: float,
              param_type: str = DEFAULT_PARAM_TYPE,
              timeout_s: float = 5.0) -> Tuple[bool, str, bool]:
        """Write `value` to `param_id`.

        Returns `(posted_ok, reason, verified)`:
          * `posted_ok`  — True iff mavlink2rest ACKed the POST as a valid
                           MAVLink message. False means the message never
                           reached the autopilot (schema rejected, network
                           dead, etc.); the write did not happen.
          * `verified`   — True iff we also observed a fresh PARAM_VALUE
                           echo whose name matches. False means the POST
                           went out but no echo arrived within the timeout;
                           the write may still have taken (echo can be
                           lost) — the follow-up Check step reveals truth.

        We snapshot the mailbox stamp BEFORE the write so a pre-existing
        PARAM_VALUE with the same name can't fool us into declaring
        success on a write that never happened.
        """
        if not self.ensure_target():
            return False, "no autopilot found on mavlink2rest", False

        message = {
            "type": "PARAM_SET",
            "target_system": self.target_system,
            "target_component": self.target_component,
            "param_id": _str_to_chars(param_id),
            "param_value": float(value),
            "param_type": {"type": param_type},
        }

        def send() -> bool:
            return self._post(
                self._envelope(message),
                f"PARAM_SET:{param_id}={value}",
            )

        with self._lock:
            _, before_stamp = self._param_value_mailbox()
            deadline = time.monotonic() + timeout_s
            if not send():
                return False, "mavlink2rest rejected PARAM_SET", False
            echoed = self._await_param_value(
                param_id, deadline, reject_stamp=before_stamp, repoke=send,
            )
        if echoed is None:
            return True, "posted; no PARAM_VALUE echo within timeout", False
        # Autopilot may clamp/round; report exact echoed value to caller
        # via a match tolerance rather than requiring exact equality.
        if _values_match(echoed, value):
            return True, "verified via PARAM_VALUE echo", True
        return True, (
            f"posted; echo shows {param_id}={echoed} (expected {value})"
        ), False

    def is_reachable(self, timeout_s: float = 1.5) -> bool:
        """True when the autopilot host answers with a recent HEARTBEAT."""
        if not self.ensure_target():
            return False
        try:
            r = self._session.get(
                f"{self.base_url}/mavlink/vehicles/{self.target_system}"
                f"/components/{self.target_component}/messages/HEARTBEAT",
                timeout=timeout_s,
            )
        except Exception:
            return False
        if r.status_code != 200:
            return False
        body = (r.text or "").strip()
        return bool(body) and body != "None"

    # -- higher-level operations ---------------------------------------

    def read_expected(self, expected: Dict[str, float]) -> Dict[str, dict]:
        """Read every expected param, resolving synonyms.

        Returns { canonical_name: {
            'target': float, 'current': float|None, 'match': bool,
            'resolved_name': str|None, 'available': bool, 'reason': str
        }}.
        `available=False` here means "no PARAM_VALUE echo received within
        the read timeout" — that can mean the param genuinely doesn't
        exist, OR that the echo transport is unreliable. The caller
        should not conclude the param is missing based on a single read.
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
                row['reason'] = 'no PARAM_VALUE echo within timeout'
            out[name] = row
        return out

    def apply_expected(self, expected: Dict[str, float]) -> Dict[str, dict]:
        """Write every expected param via mavlink2rest.

        Actions used:
          * `noop`              — pre-read confirmed the value already matches.
          * `wrote`             — POST accepted AND fresh PARAM_VALUE echo
                                  verified the new value.
          * `wrote_unverified`  — POST accepted, but no matching PARAM_VALUE
                                  echo arrived. Treated as `ok=True`; the
                                  Check step surfaces truth.
          * `failed`            — mavlink2rest rejected the PARAM_SET (bad
                                  schema, transport dead, etc.). The
                                  write did NOT happen.

        Returns {canonical_name: {target, previous, current, action, ok,
            reason, resolved_name, available}}.
        """
        out: Dict[str, dict] = {}
        # Re-discover the autopilot up front rather than trusting a cached
        # address. The vehicle's MAV_SYSID (SYSID_THISMAV on older
        # firmware) is operator-settable and changes on reboot after an
        # edit; a stale target would let every PARAM_SET POST succeed at
        # the HTTP layer while addressing a system nothing is listening
        # on — which is exactly the failure this whole module hit.
        if not self.ensure_target(force=True):
            reason = 'no ArduPilot/PX4 autopilot found on mavlink2rest'
            return {
                name: {
                    'target': float(target), 'previous': None,
                    'current': None, 'action': 'failed', 'ok': False,
                    'reason': reason, 'resolved_name': name,
                    'available': False,
                }
                for name, target in expected.items()
            }
        for name, target in expected.items():
            row = {
                'target': float(target),
                'previous': None,
                'current': None,
                'action': 'failed',
                'ok': False,
                'reason': '',
                'resolved_name': None,
                'available': False,
            }
            # Best-effort synonym resolution + pre-read. If neither read
            # returns, keep the canonical name and press on with the write.
            aliases = _param_synonyms(name)
            resolved: str = aliases[0]
            previous: Optional[float] = None
            for alias in aliases:
                got = self.read(alias, timeout_s=1.5)
                if got is not None:
                    resolved = alias
                    previous = got
                    row['available'] = True
                    row['previous'] = previous
                    row['current'] = previous
                    break
            row['resolved_name'] = resolved

            if previous is not None and _values_match(previous, target):
                row['action'] = 'noop'
                row['ok'] = True
                out[name] = row
                log.info("param %s already %s (noop)", resolved, previous)
                continue

            posted, why, verified = self.write(resolved, target)
            if not posted:
                row['action'] = 'failed'
                row['ok'] = False
                row['reason'] = why
                log.warning("param %s write failed: %s", resolved, why)
                out[name] = row
                continue
            row['ok'] = True
            row['current'] = float(target)
            if verified:
                row['action'] = 'wrote'
                row['available'] = True
                log.info("param %s wrote %s (verified)", resolved, target)
            else:
                row['action'] = 'wrote_unverified'
                row['reason'] = why
                log.info(
                    "param %s wrote %s (unverified; %s)",
                    resolved, target, why,
                )
            out[name] = row
        return out


# ── State helpers (pure functions; safe to unit-test) ────────────────

def snapshot_from_apply_result(result: Dict[str, dict]) -> Dict[str, float]:
    """Build the persisted `expected` snapshot from an apply result.

    Include every param whose POST succeeded (`ok=True`), verified or not.
    Unverified writes are included so the drift check has something to
    compare against later — the whole point of the check step is to reveal
    whether an unverified write actually landed. Params whose POST hard-
    failed are excluded so we don't nag about them forever.
    """
    snap: Dict[str, float] = {}
    for name, row in (result or {}).items():
        if not row.get('ok'):
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
            # Parameter vanished (firmware downgrade, etc.) — treat as
            # drift so the user sees it, but flag as unavailable.
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
    'DEFAULT_BASE_URL',
    'MIN_SERIAL_INDEX',
    'MAX_SERIAL_INDEX',
    'SERIAL_PROTOCOL_GPS',
    'SERIAL_PROTOCOL_WINDVANE',
    'WNDVN_TYPE_NMEA',
    'GPS_TYPE_NMEA',
    'GPS_TYPE_AUTO',
    'EK3_YAW_GPS',
    'EK3_YAW_GPS_WITH_COMPASS_FALLBACK',
    'EK3_POSXY_GPS',
    'EK3_VELXY_GPS',
    'validate_selection',
    'build_expected_params',
    'snapshot_from_apply_result',
    'diff_current_vs_expected',
]
