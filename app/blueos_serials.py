"""
BlueOS ArduPilot serial-port configuration for the Airmar-WX extension.

On Linux autopilot boards (Navigator) ArduPilot runs as a process that
BlueOS's ardupilot-manager launches with `-C /dev/ttyS0 -G udpin:...`
style arguments. The "Serial Port Configuration" panel on BlueOS's
Autopilot Firmware page (pirate mode) edits that list; this module drives
the same HTTP API so the extension can point SERIAL6/7 at its UDP feeds
without the user copy-pasting strings.

API facts (BlueOS 1.4.x, `core/services/ardupilot_manager`):

* `GET  /serials` -> `[{"port": "G", "endpoint": "udpin:0.0.0.0:27001"}, ...]`.
  Returns the board defaults until a list has been saved once.
* `PUT  /serials` with the FULL list. It replaces, never merges, so we
  always GET, edit, and PUT the whole list back. Any invalid entry makes
  FastAPI answer 422 and nothing is saved. `/dev/...` entries must exist.
* `PUT` does not restart anything. `POST /restart` kills ArduPilot (after
  sending it a disarm) and relaunches it with the new arguments; it can
  take tens of seconds and telemetry drops meanwhile.
* Letters map to SERIALn as below; `A` (SERIAL0) is reserved by BlueOS for
  its MAVLink router and SERIAL8+ is rejected by the backend.
* Only Linux boards use the list. USB flight controllers (Pixhawk) return
  `[]` and SITL ignores it, so auto-setup is unavailable there.

Everything here is either a pure function (safe to unit-test) or a client
method that logs and returns a failure value instead of raising.
"""

import logging
import os
import re
from typing import Dict, List, Optional, Tuple

import requests

log = logging.getLogger(__name__)

DEFAULT_BASE_URL = "http://127.0.0.1/ardupilot-manager/v1.0"

SERIAL_TO_LETTER = {1: "C", 2: "D", 3: "B", 4: "E", 5: "F", 6: "G", 7: "H"}
LETTER_TO_SERIAL = {v: k for k, v in SERIAL_TO_LETTER.items()}
MIN_AUTO_SERIAL = 1
MAX_AUTO_SERIAL = 7

# Boards where ardupilot-manager turns the serial list into ArduPilot
# command-line arguments (`start_linux_board`).
LINUX_BOARDS = ("Navigator", "Navigator64", "Argonot")

_UDP_ENDPOINT_RE = re.compile(r"^(udp|udpin):([^:]+):(\d+)$")


def udpin_endpoint(port: int) -> str:
    return f"udpin:0.0.0.0:{int(port)}"


def endpoint_udp_port(endpoint) -> Optional[int]:
    """UDP port of a `udp:`/`udpin:` endpoint, else None (devices, tcp)."""
    m = _UDP_ENDPOINT_RE.match(str(endpoint or "").strip())
    return int(m.group(3)) if m else None


def serial_label(letter: str) -> str:
    n = LETTER_TO_SERIAL.get(letter)
    return f"SERIAL{n}" if n is not None else f"-{letter}"


def is_supported(board, serials) -> Tuple[bool, str]:
    """Whether the vehicle's serial list can be configured automatically."""
    if not isinstance(board, dict):
        return False, "BlueOS autopilot manager not reachable"
    name = str(board.get("name") or "")
    if name not in LINUX_BOARDS:
        return False, (
            f"autopilot board '{name or 'unknown'}' does not use BlueOS "
            "serial port configuration"
        )
    if not isinstance(serials, list):
        return False, "could not read the BlueOS serial port configuration"
    return True, ""


def normalize_serials(entries) -> List[dict]:
    """Clean `[{port, endpoint}]` from the API; drops malformed rows."""
    out = []
    for e in entries or []:
        if not isinstance(e, dict):
            continue
        port = str(e.get("port") or "").strip()
        endpoint = str(e.get("endpoint") or "").strip()
        if port and endpoint:
            out.append({"port": port, "endpoint": endpoint})
    return out


def dead_device_entries(entries, exists=None) -> List[dict]:
    """`/dev/...` entries whose device is missing (BlueOS would 422 them)."""
    exists = exists or os.path.exists
    return [e for e in normalize_serials(entries)
            if e["endpoint"].startswith("/dev/") and not exists(e["endpoint"])]


def validate_auto_serials(wind_serial, gps_serial) -> Tuple[bool, str]:
    """Auto-setup can only target SERIAL1..7 (BlueOS letters C..H)."""
    for label, n in (("wind", wind_serial), ("GPS", gps_serial)):
        if n is None:
            continue
        if n not in SERIAL_TO_LETTER:
            return False, (
                f"SERIAL{n} for {label} can't be set automatically; BlueOS "
                f"configures SERIAL{MIN_AUTO_SERIAL}-{MAX_AUTO_SERIAL} only"
            )
    return True, ""


def plan_serial_changes(current, wind_serial, gps_serial,
                        wind_endpoint, gps_endpoint) -> Tuple[List[dict], List[dict]]:
    """Plan the serial list that routes our UDP feeds to the chosen serials.

    Returns `(new_list, changes)`. Each change is
    `{serial, letter, route, kind, before, after}` where kind is:

    * `set`        — target letter was empty.
    * `replaced`   — target letter held something else (before = old value).
    * `moved-from` — our UDP port was on another letter; removed there, since
                     two serials can't listen on the same port.
    * `unchanged`  — target letter already had our endpoint.

    Entries we don't touch keep their order; new letters are appended.
    """
    entries = normalize_serials(current)
    by_letter: Dict[str, str] = {}
    order: List[str] = []
    for e in entries:
        if e["port"] not in by_letter:
            order.append(e["port"])
        by_letter[e["port"]] = e["endpoint"]

    routes = []
    if wind_serial is not None:
        routes.append(("wind", SERIAL_TO_LETTER[wind_serial], wind_endpoint))
    if gps_serial is not None:
        routes.append(("gps", SERIAL_TO_LETTER[gps_serial], gps_endpoint))
    targets = {letter for _, letter, _ in routes}

    changes: List[dict] = []
    for route, letter, endpoint in routes:
        port = endpoint_udp_port(endpoint)
        for other in list(order):
            if other == letter or other in targets or other == "A":
                continue
            if port is not None and endpoint_udp_port(by_letter.get(other)) == port:
                changes.append({
                    "serial": LETTER_TO_SERIAL.get(other), "letter": other,
                    "route": route, "kind": "moved-from",
                    "before": by_letter[other], "after": None,
                })
                del by_letter[other]
                order.remove(other)
        before = by_letter.get(letter)
        if before == endpoint:
            kind = "unchanged"
        elif before is None:
            kind = "set"
            order.append(letter)
        else:
            kind = "replaced"
        by_letter[letter] = endpoint
        changes.append({
            "serial": LETTER_TO_SERIAL[letter], "letter": letter,
            "route": route, "kind": kind, "before": before, "after": endpoint,
        })

    new_list = [{"port": p, "endpoint": by_letter[p]} for p in order]
    return new_list, changes


def plan_serial_undo(current, snapshot, exists=None
                     ) -> Tuple[List[dict], List[dict]]:
    """Plan the serial list that reverts what auto-setup changed.

    `snapshot` is `{letter: {before, after, route}}`. A letter is reverted
    only while it still holds what we wrote; if the user has changed it
    since, it is left alone (`skipped_changed`). A `before` device that no
    longer exists can't be put back (`skipped_missing_device`).

    Returns `(new_list, rows)`, rows `{serial, letter, kind, before, after,
    current}` with kind `restore` | `remove` | `unchanged` |
    `skipped_changed` | `skipped_missing_device`. In a row, `before` is the
    value now and `after` the value Undo leaves.
    """
    exists = exists or os.path.exists
    entries = normalize_serials(current)
    by_letter: Dict[str, str] = {}
    order: List[str] = []
    for e in entries:
        if e["port"] not in by_letter:
            order.append(e["port"])
        by_letter[e["port"]] = e["endpoint"]

    rows: List[dict] = []
    for letter in sorted(snapshot or {}, key=lambda l: LETTER_TO_SERIAL.get(l, 99)):
        rec = snapshot[letter] or {}
        original, wrote = rec.get("before"), rec.get("after")
        now = by_letter.get(letter)
        row = {"serial": LETTER_TO_SERIAL.get(letter), "letter": letter,
               "before": now, "after": now, "original": original}
        if original == wrote or now == original:
            row["kind"] = "unchanged"
        elif now != wrote:
            row["kind"] = "skipped_changed"
        elif original is not None and original.startswith("/dev/") and not exists(original):
            row["kind"] = "skipped_missing_device"
        elif original is None:
            row["kind"] = "remove"
            row["after"] = None
            del by_letter[letter]
            order.remove(letter)
        else:
            row["kind"] = "restore"
            row["after"] = original
            if letter not in by_letter:
                order.append(letter)  # setup removed it (moved-from)
            by_letter[letter] = original
        rows.append(row)

    new_list = [{"port": p, "endpoint": by_letter[p]} for p in order]
    return new_list, rows


class SerialsClient:
    """Thin client for BlueOS ardupilot-manager. Never raises."""

    def __init__(self, base_url: str = DEFAULT_BASE_URL,
                 http_timeout_s: float = 5.0,
                 put_timeout_s: float = 10.0,
                 restart_timeout_s: float = 90.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.http_timeout_s = http_timeout_s
        self.put_timeout_s = put_timeout_s
        self.restart_timeout_s = restart_timeout_s
        self._session = requests.Session()

    def _get(self, path):
        url = f"{self.base_url}{path}"
        try:
            r = self._session.get(url, timeout=self.http_timeout_s)
        except Exception as e:
            log.debug("ardupilot-manager GET %s failed: %s", url, e)
            return None
        if r.status_code != 200:
            log.debug("ardupilot-manager GET %s -> HTTP %s", url, r.status_code)
            return None
        try:
            return r.json()
        except Exception as e:
            log.debug("ardupilot-manager GET %s json decode failed: %s", url, e)
            return None

    def get_board(self) -> Optional[dict]:
        board = self._get("/board")
        return board if isinstance(board, dict) else None

    def get_serials(self) -> Optional[List[dict]]:
        serials = self._get("/serials")
        if not isinstance(serials, list):
            return None
        return normalize_serials(serials)

    def put_serials(self, entries) -> Tuple[bool, str]:
        body = normalize_serials(entries)
        try:
            r = self._session.put(f"{self.base_url}/serials", json=body,
                                  timeout=self.put_timeout_s)
        except Exception as e:
            return False, f"could not reach BlueOS autopilot manager: {e}"
        if r.status_code != 200:
            return False, (f"BlueOS rejected the serial list (HTTP {r.status_code}): "
                           f"{(r.text or '')[:300]}")
        return True, "saved"

    def restart(self) -> Tuple[bool, str]:
        """Restart ArduPilot. `(True, 'timeout')` when the call timed out:
        BlueOS may still be restarting, so the caller waits for heartbeats."""
        try:
            r = self._session.post(f"{self.base_url}/restart",
                                   timeout=self.restart_timeout_s)
        except Exception as e:
            if "timed out" in str(e).lower() or "timeout" in type(e).__name__.lower():
                return True, "timeout"
            return False, f"could not reach BlueOS autopilot manager: {e}"
        if r.status_code != 200:
            return False, (f"BlueOS restart failed (HTTP {r.status_code}): "
                           f"{(r.text or '')[:300]}")
        return True, "restarted"
