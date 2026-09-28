# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A BlueOS extension (Docker image `vshie/blueos-airmar-wx`) for Airmar WX-series weather stations (300WX, legacy 200WX). It reads NMEA 0183 over USB serial and fans the data out to: a Vue/Vuetify web UI (port 6436), two UDP NMEA feeds into ArduPilot, `NAMED_VALUE_FLOAT` messages via mavlink2rest, and a Cockpit data-lake WebSocket (port 8765). README.md is the user-facing spec and explains the *why* behind most networking/parameter decisions — read the relevant section before changing that behaviour.

## Commands

```bash
# Run all tests (stdlib unittest, no pytest; no runtime deps needed — see tests/_stubs.py)
python3 -m unittest discover -s tests -t .

# Single file / single test
python3 -m unittest tests.test_udp_routing
python3 -m unittest tests.test_udp_routing.<TestClass>.<test_method>

# Build the image
docker build -t vshie/blueos-airmar-wx:latest .
```

There is no linter config and no docker-compose file. CI (`.github/workflows/deploy.yml`) builds and pushes the multi-arch image on every push via `BlueOS-community/Deploy-BlueOS-Extension`; it does not run tests.

Frontend vendor assets (Vue 2.7, Vuetify 2.7, axios, MDI, Roboto) are downloaded into `app/static/vendor/` by the Dockerfile at build time, so the UI only works fully inside the built image.

## Versioning

Each change ships as a patch bump, and the version lives in three places that must stay in sync: `Dockerfile` (`LABEL org.blueos.version` and `LABEL version`) and `app/pyproject.toml`. Commit messages end with the version in parentheses, e.g. `... (1.1.8)`.

## Architecture

- `app/main.py` — nearly everything. A single `NMEAHandler` instance (module global `nmea_handler`) owns all state and threads; Flask routes at the bottom of the file are thin wrappers around its methods. Served by waitress with 16 threads because each SSE client (`/api/events`) pins a worker.
  - Threads started from `NMEAHandler.__init__`: serial auto-connect (`_auto_connect`, reconnects only to the port saved in `state.json`, resolved via its `/dev/serial/by-id` name; it deliberately never scans other ports because probing sends Airmar commands and changes baud on whatever is attached, e.g. the vehicle GPS), serial reader (`_read_serial_loop` → `_parse_nmea_for_dashboard` → history buffers, SSE broadcast, UDP routing, WebSocket broadcast), mavlink NVF publisher (`_mav_publish_loop`, 1 Hz, always on), and the Cockpit WebSocket server.
  - Connect flow: UI connects go through `user_connect`, which cancels any in-flight auto-connect and waits on `_connect_lock` before clearing `_cancel_connect`. `connect_serial` probes baud (`_try_baud_rate`), switches the sensor to 115200 with `$PAMTC,BAUD,...,CFG` (`_switch_to_operating_baud`), queries device info/model, and enables required sentences at intervals sized to the baud rate.
  - UDP routing: `_route_for_sentence` sends MWV → 27001 (wind) and GGA/RMC/VTG/HDT → 27002 (GPS), matching on sentence ID regardless of talker prefix. Each route is opt-in via persisted `state['stream_wind']` / `state['stream_gps']` (set by `set_stream_routes`, `POST /api/stream/routes`); `is_streaming` is derived from them. Connecting never enables streaming. Both routes go out one socket bound to `127.0.0.1:27100`; ArduPilot's `udpin` pins the first peer's address *and* port, so the source must never change. `_maybe_drain_udp_socket` drains replies the autopilot sends back to that socket.
  - Persistence: `state.json` in the log dir (`/app/logs`, bind-mounted to `/usr/blueos/extensions/300WX`) holds last port/baud and the autopilot-setup snapshot. `load_state` includes migrations (e.g. `_migrate_autopilot_setup_gps_contract`) — when changing persisted shape, add a migration and a test in `tests/test_state_migration.py`.
- `app/mavlink_params.py` — ArduRover parameter apply/check/drift-diff over mavlink2rest (`http://127.0.0.1/mavlink2rest`). Key constraints documented in the module docstring: mavlink2rest returns HTTP 200 on malformed messages (must check body), message bodies must be built from the server's `/helper/mavlink?name=` template, PARAM_VALUE freshness is judged by `status.time.last_update`. The autopilot's system ID is discovered (`pick_autopilot`), never assumed to be 1. `build_expected_params` is the single source of the parameter contract (GPS1 stays AUTO=1, Airmar is GPS2=NMEA 5); missing firmware params are skipped, not failures.
- `app/mavlink_sender.py` — `NAMED_VALUE_FLOAT` publisher (`WX_AppDir/AppSpd/TruDir/TruSpd`). Uses the `WX_` prefix deliberately to avoid colliding with ArduPilot's own `AppWnd*` NVFs, and a distinct component ID per value (70–73, system 255) so mavlink-server's cache keeps all four. Only publishes values fresher than 5 s — never sends stale values as zero.
- `app/static/index.html` — the whole UI as a single-file Vue 2 + Vuetify app (no build step); `widget.html` is an embeddable wind/nav widget served at `/widget`.

## Tests

Tests import `app.main` directly after `tests/_stubs.py` installs fake `flask`, `serial`, `requests`, `waitress`, `websockets` modules into `sys.modules`, and redirect the log dir to a tempdir. New tests should follow the same pattern (`from tests import _stubs; _stubs.install()` before importing app code) and use fake sockets / fake mavlink2rest responses rather than real I/O.
