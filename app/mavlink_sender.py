"""
Send NAMED_VALUE_FLOAT messages from the Airmar-WX extension to the autopilot
via mavlink2rest.

Why this exists
---------------
ArduPilot logs every NAMED_VALUE_FLOAT it receives to the .BIN DataFlash log
as an `NVF` record. Pushing the apparent / true wind values through here makes
them line up timestamp-for-timestamp with the rest of the flight data (GPS,
ATT, MODE, WIND, etc.) without needing a separate post-flight join against
the extension's own log file. The first version of this extension only
forwarded `$WIMWV` over UDP NMEA (port 27000), which is consumed by ArduPilot's
`AP_WindVane_NMEA` driver -- but only when the wind vane is configured
(`WNDVN_TYPE = 4 / WNDVN_SPEED_TYPE = 4` on Rover) AND the UDP path actually
lands on the autopilot's NMEA serial. If either is wrong, the wind-vane
subsystem stays at zero and `WIND.SpdApp` / `WIND.DrApp` log as 0.0.

Important: do NOT reuse ArduPilot's own NVF names here. `AP_WindVane.cpp`
emits these from `AP_WindVane::send_wind()`:

    gcs().send_named_float("AppWndSpd", get_apparent_wind_speed());
    gcs().send_named_float("AppWndDir", degrees(get_apparent_wind_direction_rad()));

so any extension that POSTs `AppWndSpd` / `AppWndDir` to mavlink2rest from a
different `component_id` produces a duplicate stream in the .BIN log under the
same NVF name, and any 0.0s it sends are indistinguishable from the
autopilot's own "wind vane is empty" emissions. We therefore use the
`WX_*` prefix below; the resulting NVFs sit alongside `AppWndSpd` / `AppWndDir`
in the log and the inspector and serve as ground truth from the sensor that
is independent of the wind-vane subsystem's state.

In the reference .BIN trace (`Downloads/00000230.BIN`) every `WIND.SpdApp` /
`WIND.DrApp` and every NVF `AppWndSpd` / `AppWndDir` is 0.0 -- consistent with
`$WIMWV` never reaching the autopilot's wind-vane driver, not with a buggy
sender in this extension (the repo has never shipped one). This module is
the additive path that bypasses that pipeline entirely.

Pattern (mirrors `Mikrotik-Monitor/app/mavlink_sender.py`)
---------------------------------------------------------
* mavlink-server (BlueOS's mavlink2rest fork) stores by
  `(system_id, component_id, message_type)`, so multiple NAMED_VALUE_FLOATs
  posted from the same component_id overwrite each other in the GET / web
  inspector view -- only the last write survives. Each metric therefore
  gets its own `component_id = base + NAMED_VALUE_OFFSETS[name]` so the
  inspector and any per-name subscribers can see all of them simultaneously.
* The autopilot still logs every POST regardless of component_id (each becomes
  its own mavlink packet on the wire), so this isn't required for .BIN logging
  alone -- but it keeps the Cockpit / mavlink-inspector experience correct.
* `name` must be a 10-element list of 1-char strings, null-padded, per
  mavlink2rest's wire format expectation. Sending `name` as a plain string
  silently gets rejected.

Endpoint discovery
------------------
BlueOS has historically exposed mavlink2rest under several paths. Because
the container runs with NetworkMode=host, `127.0.0.1` is the host and is
the preferred address; `host.docker.internal` (docker bridge gateway) is
kept only as a fallback for older BlueOS builds.
  - `http://127.0.0.1:6040/v1/mavlink`                   (direct, current)
  - `http://127.0.0.1/mavlink2rest/mavlink`              (NGINX proxy)
  - `http://192.168.2.2/mavlink2rest/mavlink`            (vehicle IP fallback)
  - `http://host.docker.internal:6040/v1/mavlink`        (legacy compat)
We try them on the first POST, cache the first one that returns 2xx, and
fall back through the list again if the cached one starts failing.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Iterable, List, Optional

import requests

log = logging.getLogger('app')


# Component-ID layout. Default base 70 sits clear of the BlueOS PH/TEMP/
# SALINITY/CONDUCT extension (25-28) and the Mikrotik-Monitor extension
# (default base 60, occupies 60..66 with the heartbeats). Bump
# NAMED_VALUE_COMPONENT_BASE if another extension claims this range.
NAMED_VALUE_COMPONENT_BASE = 70

# NVF names are limited to 10 chars on the wire. The `WX_` prefix marks the
# Airmar WeatherStation extension as the origin and -- crucially -- keeps these
# names distinct from `AppWndSpd` / `AppWndDir`, which ArduPilot's AP_WindVane
# itself emits from `AP_WindVane::send_wind()`. Co-existing in the .BIN log
# makes it possible to compare extension-direct values against the wind-vane
# subsystem's view and diagnose UDP-NMEA / `WNDVN_TYPE` misconfiguration.
NAMED_VALUE_OFFSETS = {
    'WX_AppDir': 0,  # Apparent wind direction, bow-relative (deg, 0..360)
    'WX_AppSpd': 1,  # Apparent wind speed (knots)
    'WX_TruDir': 2,  # True wind direction (deg; north-ref when $WIMWD, bow-relative when $WIMWV-T)
    'WX_TruSpd': 3,  # True wind speed (knots)
}

HEADER_SYSTEM_ID = 255  # GCS-style sender, same as Odometer / Mikrotik-Monitor

# Endpoint candidates, in preference order. The first 2xx response wins and is
# cached for the remainder of the process lifetime. Localhost first because
# the container runs with NetworkMode=host; `host.docker.internal` is only
# kept for legacy BlueOS builds where the loopback route may not exist.
POST_ENDPOINTS = (
    'http://127.0.0.1:6040/v1/mavlink',
    'http://127.0.0.1/mavlink2rest/mavlink',
    'http://localhost:6040/v1/mavlink',
    'http://localhost/mavlink2rest/mavlink',
    'http://192.168.2.2:6040/v1/mavlink',
    'http://192.168.2.2/mavlink2rest/mavlink',
    'http://host.docker.internal:6040/v1/mavlink',
    'http://host.docker.internal/mavlink2rest/mavlink',
    'http://blueos.local:6040/v1/mavlink',
    'http://blueos.local/mavlink2rest/mavlink',
)

# Per-POST timeout. mavlink2rest is local so this only ever trips when the
# extension framework is wedged; we keep it short to avoid stalling the sender
# loop and falling behind real-time.
POST_TIMEOUT_S = 2.0

# Maximum age (seconds) of the underlying NMEA parse for a value to still be
# considered "fresh". The 300WX emits MWV at 10 Hz by default, so 5 s is two
# orders of magnitude beyond expected cadence -- anything older indicates the
# serial link has dropped and we should NOT publish a stale value.
DEFAULT_FRESH_S = 5.0


def _nvf_name_field(name: str) -> List[str]:
    """Encode a NAMED_VALUE_FLOAT name as 10 single-char strings, null-padded.

    mavlink2rest expects `name` as a 10-element list of single-character
    strings (one Python string per byte slot). Passing a plain string is
    silently accepted by some versions and rejected by others -- always send
    the list form.
    """
    out: List[str] = []
    for i in range(10):
        out.append(name[i] if i < len(name) else '\x00')
    return out


def _nvf_payload(name: str, value: float, header_component_id: int) -> dict:
    return {
        'header': {
            'system_id': HEADER_SYSTEM_ID,
            'component_id': header_component_id,
            'sequence': 0,
        },
        'message': {
            'type': 'NAMED_VALUE_FLOAT',
            'time_boot_ms': 0,
            'value': float(value),
            'name': _nvf_name_field(name),
        },
    }


class MavlinkSender:
    """Sync, thread-safe sender that caches the first working POST endpoint.

    One instance per process; reused by the 1 Hz publish thread. Keeps a
    `requests.Session` for connection reuse, and only escalates a failed
    POST to "log a warning" once per `_warn_interval_s` to avoid flooding
    300wx.log when the autopilot framework is down.
    """

    def __init__(
        self,
        endpoints: Iterable[str] = POST_ENDPOINTS,
        timeout_s: float = POST_TIMEOUT_S,
        component_id_base: int = NAMED_VALUE_COMPONENT_BASE,
    ):
        self._endpoints = tuple(endpoints)
        self._timeout_s = timeout_s
        self._component_id_base = component_id_base
        self._session = requests.Session()
        self._cached_endpoint: Optional[str] = None
        self._lock = threading.Lock()
        self._last_warn_ts = 0.0
        self._warn_interval_s = 30.0

    def planned_component_ids(self) -> dict:
        """For diagnostics: name -> component_id this sender will use."""
        return {
            name: self._component_id_base + offset
            for name, offset in NAMED_VALUE_OFFSETS.items()
        }

    def _candidates(self) -> List[str]:
        """Return endpoints in preference order, putting the cached one first."""
        cached = self._cached_endpoint
        if cached is None:
            return list(self._endpoints)
        return [cached] + [e for e in self._endpoints if e != cached]

    def _post_one(self, payload: dict) -> bool:
        """POST `payload` to the first endpoint that accepts it; cache it."""
        last_error: Optional[str] = None
        for endpoint in self._candidates():
            try:
                r = self._session.post(endpoint, json=payload, timeout=self._timeout_s)
            except Exception as e:
                last_error = f"{endpoint}: {e}"
                continue
            if 200 <= r.status_code < 300:
                # Cache this endpoint so future POSTs go straight to the
                # working one. We hold the lock only for the assignment so
                # concurrent senders don't tear the cached value.
                with self._lock:
                    if self._cached_endpoint != endpoint:
                        log.info("mavlink2rest endpoint locked in: %s", endpoint)
                        self._cached_endpoint = endpoint
                return True
            last_error = f"{endpoint}: HTTP {r.status_code} {r.text[:120]!s}"
            # Non-2xx: drop the cache so the next POST re-probes the list.
            with self._lock:
                if self._cached_endpoint == endpoint:
                    self._cached_endpoint = None

        # All endpoints failed -- throttle the warning.
        now = time.monotonic()
        if now - self._last_warn_ts >= self._warn_interval_s:
            self._last_warn_ts = now
            log.warning(
                "mavlink2rest NVF POST failed on every endpoint; last error: %s. "
                "Is mavlink2rest running on the host? (tried %d endpoints)",
                last_error, len(self._endpoints),
            )
        return False

    def send_named_value_floats(self, values: dict) -> int:
        """POST one NAMED_VALUE_FLOAT per (name, value) entry. Skips None/NaN.

        Returns the number of NVFs successfully POSTed (HTTP 2xx). Each metric
        is POSTed independently because mavlink2rest's storage and the wire
        format are one-message-per-request; the autopilot logs each as its
        own NVF row.
        """
        sent = 0
        for name, raw_value in values.items():
            if raw_value is None:
                continue
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                continue
            # NaN guard -- NamedValueFloat with NaN turns into a noise row in
            # the .BIN log that's harder to filter out than a missing entry.
            if value != value:  # noqa: PLR0124 (NaN check is intentional)
                continue
            offset = NAMED_VALUE_OFFSETS.get(name)
            if offset is None:
                # Unknown name -- still send it, but from the base component
                # so we don't silently drop user-added metrics.
                offset = 0
            component_id = self._component_id_base + offset
            if self._post_one(_nvf_payload(name, value, component_id)):
                sent += 1
        return sent


__all__ = [
    'HEADER_SYSTEM_ID',
    'MavlinkSender',
    'NAMED_VALUE_COMPONENT_BASE',
    'NAMED_VALUE_OFFSETS',
    'DEFAULT_FRESH_S',
]
