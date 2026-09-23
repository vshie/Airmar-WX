# Airmar 300WX WeatherStation - BlueOS Extension

A BlueOS extension for the Airmar 300WX WeatherStation. Connects via NMEA 0183 serial, auto-negotiates **115200** baud (with `$PAMTC,BAUD,...,CFG` so the default persists across power cycles), and streams wind data to Cockpit via WebSocket.

## Features

- Automatic serial port detection and baud rate negotiation (4800 → 115200, default saved on device)
- **Auto-reconnect on extension restart**: last serial port, baud hint, and “stay at 4800” preference are stored in `state.json` under the mounted logs directory; a background thread connects at startup without using the UI. **Disconnect** in the UI clears the saved port so the next restart will scan ports instead.
- Real-time dashboard with wind, heading, atmosphere, GPS, and attitude data (including apparent/true wind roses and speed–time heatmaps)
- Sparkline history graphs for all sensor channels
- Per-sentence enable/disable and transmission interval control
- Bandwidth usage indicator (percentage of serial bus capacity)
- Raw message view with one card per message type and live Hz rate
- Dual UDP streams to ArduPilot — wind on `27001` and GPS + heading on `27002`, always active while streaming is on
- One-click ArduRover parameter setup (SERIAL X/Y, wind vane + GPS NMEA + optional GPS-yaw source) with drift detection and restore/ignore
- Cockpit data-lake WebSocket streaming of wind, GPS, and heading data
- Persistent NMEA message and application logs with download/delete

## Installation

### From BlueOS Extensions Manager

1. Open BlueOS web interface
2. Navigate to Extensions Manager
3. Search for "Airmar 300WX"
4. Click Install

### Manual Install

1. In BlueOS, go to Extensions Manager > Installed > "+"
2. Enter:
   - **Extension Identifier**: `bluerobotics.airmar-300wx`
   - **Extension Name**: `Airmar 300WX`
   - **Docker image**: `vshie/blueos-airmar-wx`
   - **Docker tag**: `main`

## Custom Settings (Permissions)

When manually installing, paste this into the **Custom settings** field:

```json
{
  "ExposedPorts": {
    "6436/tcp": {},
    "8765/tcp": {}
  },
  "HostConfig": {
    "CpuPeriod": 100000,
    "CpuQuota": 100000,
    "Binds": [
      "/usr/blueos/extensions/300WX:/app/logs",
      "/dev:/dev"
    ],
    "NetworkMode": "host",
    "Privileged": true
  }
}
```

> **Networking note.** `NetworkMode: host` is required so the two NMEA UDP
> feeds (`27001` wind, `27002` GPS + heading) can be sent from `127.0.0.1`
> to the autopilot's `udpin` sockets. The earlier design routed via the
> docker bridge (`host.docker.internal` / 172.18.0.1); ArduPilot's
> `UDPDevice::read()` calls `socket.connect()` on the first datagram's
> source address, which was 192.168.2.12 (the host's LAN address). After
> that, every subsequent datagram — still addressed to 172.18.0.1 — was
> silently dropped as "not from the connected peer". Sending from the
> loopback keeps the source address stable and works because
> `NetworkMode: host` makes the container share the host's `lo`. The
> `ExtraHosts` mapping for `host.docker.internal` is no longer required
> and has been removed.

## ArduPilot UDP Streaming (dual routes)

The extension forwards NMEA sentences to the autopilot via **two** UDP ports at the same time so both the wind vane driver and the GPS/heading driver can be fed simultaneously (each ArduPilot serial has a single `SERIALx_PROTOCOL`, so one port cannot serve both drivers).

| Route | UDP port | Sentences | ArduPilot driver |
|---|---|---|---|
| Wind    | `27001` | `$WIMWV`                                | `AP_WindVane_NMEA` (`WNDVN_TYPE = 4`) |
| GPS + heading | `27002` | `$GPGGA`, `$GPRMC`, `$GPVTG`, `$HCHDT` | `AP_GPS` NMEA driver on **GPS2** (`GPS2_TYPE = 5`); onboard GPS stays on `GPS1_TYPE = 1` (AUTO) |

### BlueOS serial port configuration (manual)

Open **BlueOS → Autopilot Firmware → Serial port configuration** and set two unused serial slots to the corresponding `udpin` device string, then Save and Restart the autopilot:

- `udpin:0.0.0.0:27001` — the SERIAL port that will act as the wind vane
- `udpin:0.0.0.0:27002` — the SERIAL port that will act as the GPS/heading source

Do NOT use the same serial index for both, and do NOT overwrite the on-board GPS serial. The extension surfaces both strings with a click-to-copy button on the **Sentences** tab (next to a screenshot of the serial-config page).

### ArduRover parameter setup (from the extension)

In **Setup → Step 2b**, pick the two SERIAL indexes you assigned above (e.g. `SERIAL2` and `SERIAL7`) and click **Apply parameters**. The extension writes the following values via mavlink2rest, verified by a fresh `PARAM_VALUE` echo per param:

| Param                        | Value | Purpose |
|---|---|---|
| `SERIAL{X}_PROTOCOL`         | `21` | Wind serial → WindVane |
| `WNDVN_TYPE`                 | `4`  | NMEA wind vane |
| `WNDVN_SPEED_TYPE`           | `4`  | NMEA wind speed (else speed defaults to none) |
| `SERIAL{Y}_PROTOCOL`         | `5`  | GPS serial → GPS |
| `GPS1_TYPE` (or `GPS_TYPE`)  | `1`  | **AUTO** — leaves the BlueBoat's onboard u-Blox on GPS1. Legacy `GPS_TYPE` synonym used if `GPS1_TYPE` is absent. |
| `GPS2_TYPE`                  | `5`  | NMEA — the Airmar's NMEA-over-UDP stream becomes GPS2. |
| `EK3_SRC2_YAW`               | `2`  | Alternate EKF source set uses GPS yaw |
| `EK3_SRC2_POSXY`             | `3`  | Alternate EKF source set uses GPS for horizontal position |
| `EK3_SRC2_VELXY`             | `3`  | Alternate EKF source set uses GPS for horizontal velocity |
| `EK3_SRC1_YAW` *(opt-in)*    | `3`  | Optional: promote Airmar HDT to primary yaw source with compass fallback |

> **Why GPS1 stays on AUTO, not NMEA.** ArduPilot's `AUTO` (`1`) probes
> u-Blox, SBP, SiRF, and ERB, which covers the BlueBoat's stock GPS.
> `AUTO` explicitly does *not* probe NMEA, so the Airmar must be on
> `GPS2_TYPE = 5`. Earlier versions of this extension wrote
> `GPS1_TYPE = 5`, which disabled the onboard GPS; on install of 1.1.6+
> the persisted `expected` snapshot is migrated to the two-GPS contract
> and the drift banner surfaces so the operator can re-Apply.

Notes:

- **Apply-once model.** Parameters are only written when you click Apply. A background check compares live values against the applied snapshot every 30 s; if any drift is detected, the UI offers **Restore** (re-apply) or **Ignore** (persist a per-selection "never nag again" flag). Changing the SERIAL indexes or the yaw fallback checkbox counts as a new setup and clears the ignore flag.
- **UDPIN ignores `SERIALx_BAUD`.** The extension does not write baud so a wired UART on the same index is not silently reconfigured.
- **Missing firmware params are skipped**, not treated as failures — the setup still succeeds on a build without wind-vane support, and the UI reports which rows were unavailable.
- **The extension does not write the BlueOS serial device string.** That mapping lives in BlueOS, not in ArduPilot parameters — you still paste the two `udpin` strings manually and reboot the autopilot after Save.
- **Restart the autopilot** after applying parameters that change `SERIALx_PROTOCOL`, `GPS1_TYPE`, `GPS2_TYPE`, or `WNDVN_TYPE` — those are read at boot.

See [ArduRover Wind Vane docs](https://ardupilot.org/rover/docs/wind-vane.html), [ArduPilot NMEA GPS](https://ardupilot.org/copter/docs/common-gps-how-it-works.html), and [EKF Source Selection](https://ardupilot.org/copter/docs/common-ekf-sources.html) for autopilot-side background.

#### Autopilot addressing (MAV_SYSID)

The extension does **not** assume the autopilot is at MAVLink system 1. It queries `GET /mavlink2rest/mavlink/vehicles` and picks the component whose `HEARTBEAT` reports a real autopilot (`MAV_AUTOPILOT_ARDUPILOTMEGA` or `MAV_AUTOPILOT_PX4`), preferring surface-boat / ground-rover vehicle types when several are visible. BlueOS's own onboard-controller heartbeat and this extension's `NAMED_VALUE_FLOAT` publishers are ignored.

This matters because `MAV_SYSID` (`SYSID_THISMAV` on firmware before 4.6) is operator-settable. On a BlueBoat it is commonly **2**, with system 1 holding only BlueOS's companion-computer heartbeat. Addressing `PARAM_SET` to the wrong system is silently ignored by the autopilot and is indistinguishable from "the parameter does not exist" — the discovered address is logged at startup:

```
INFO:app:Autopilot discovered at system 2 component 1
```

Discovery is refreshed at the start of every Apply, and retried automatically if a read stops returning, so changing `MAV_SYSID` and rebooting does not require reinstalling the extension.

## ArduPilot mavlink2rest NVF Streaming

Independent of the UDP NMEA stream above, the extension also publishes the latest wind values as `NAMED_VALUE_FLOAT` MAVLink messages via the BlueOS mavlink2rest endpoint at 1 Hz. These appear in the autopilot's DataFlash `.BIN` log as `NVF` rows and in the BlueOS MAVLink inspector, side-by-side with `MTK_*` / `ODO_*` etc. from other extensions.

| NVF Name | Source | Description |
|---|---|---|
| `WX_AppDir` | `$WIMWV` (R) | Apparent wind direction, bow-relative (deg) |
| `WX_AppSpd` | `$WIMWV` (R) | Apparent wind speed (knots) |
| `WX_TruDir` | `$WIMWD` if present, else `$WIMWV` (T) | True wind direction (deg, north-ref when from `$WIMWD`) |
| `WX_TruSpd` | `$WIMWD` / `$WIMWV` (T) | True wind speed (knots) |

### Why `WX_*` and not `AppWndSpd` / `AppWndDir`?

ArduPilot's `AP_WindVane::send_wind()` itself emits `AppWndSpd` and `AppWndDir` as NAMED_VALUE_FLOAT whenever the wind-vane subsystem is enabled — driven by whatever feeds `AP_WindVane_NMEA` (i.e. `$WIMWV` arriving on the autopilot's NMEA serial / UDP). If that pipeline is misconfigured (e.g. `WNDVN_TYPE` / `WNDVN_SPEED_TYPE` not set to `4` for NMEA, wrong serial port, etc.) the autopilot still emits `AppWndSpd` / `AppWndDir` — but stuck at `0.0`. That is exactly the failure mode visible in earlier `.BIN` logs.

Using the `WX_*` prefix here gives the extension its own NVF namespace, so both streams co-exist in the log:

- **`WX_*`**  → ground truth from the 300WX, independent of the autopilot wind-vane subsystem
- **`AppWnd*`** → what the autopilot's wind-vane subsystem actually sees

Post-flight, comparing the two answers a single diagnostic question: did `$WIMWV` actually reach the wind-vane driver? If `WX_AppSpd` is non-zero and `AppWndSpd` is zero, the autopilot is misconfigured; fix `WNDVN_TYPE = 4` and the NMEA serial routing.

### Implementation notes

- Values are only published when the underlying NMEA parse is fresh (< 5 s old). Stale or missing values are skipped, **not** sent as zero.
- Each NVF uses its own MAVLink `component_id` (`WX_AppDir`=70, `WX_AppSpd`=71, `WX_TruDir`=72, `WX_TruSpd`=73, all under `system_id=255`). mavlink-server's GET cache stores by `(system_id, component_id, message_type)`, so distinct component IDs are required for all four values to remain visible in the BlueOS MAVLink inspector. The autopilot logs every NVF it receives regardless.
- Publishing is always-on while the extension is running; it does **not** depend on the UDP "Start streaming" toggle.
- Diagnostics: `GET http://<vehicle>:6436/api/mavlink/nvf_status` returns publish count, planned component IDs, and the timestamp of the last successful POST.

## Cockpit WebSocket Streaming

The extension streams live wind, GPS, and heading data to Cockpit's data-lake via WebSocket (port 8765). All variables are sent **regardless of the ArduPilot UDP streaming state**.

### Variables Streamed

| Variable | Source | Description |
|---|---|---|
| `wind-direction-true` | `$WIMWD` | True wind direction relative to north (degrees) |
| `wind-speed-kts` | `$WIMWD` | True wind speed (knots) |
| `heading-true` | `$HCHDT` | True heading from compass (degrees) |
| `gps-latitude` | `$GPGGA` | GPS latitude (decimal degrees) |
| `gps-longitude` | `$GPGGA` | GPS longitude (decimal degrees) |
| `gps-altitude-m` | `$GPGGA` | GPS altitude (metres) |
| `gps-satellites` | `$GPGGA` | Number of GPS satellites in use |
| `gps-fix-quality` | `$GPGGA` | GPS fix quality (0=none, 1=GPS, 2=DGPS) |
| `gps-course-true` | `$GPVTG` | Course over ground (degrees true) |
| `gps-speed-kts` | `$GPVTG` | Speed over ground (knots) |

### Setting Up Cockpit Connection

1. Open **Cockpit > Menu > Settings > General**
2. Scroll to **Generic WebSocket connections**
3. Add the URL: `ws://{{ vehicle-address }}:8765`
   (e.g., `ws://192.168.2.2:8765`)

Once connected, all variables appear in Cockpit's data-lake and can be assigned to any widget, mini-widget, or HUD overlay.

## Log Files

Log files are stored in the BlueOS extensions directory:

- Location: `/usr/blueos/extensions/300WX/`
- Files:
  - `nmea_messages.log` — Raw NMEA message history
  - `300wx.log` — Application operational log

These logs persist across container restarts and can be managed from the Logs tab in the extension UI.

## Usage

1. Open the Airmar 300WX extension from the BlueOS sidebar
2. The extension auto-connects to the last used serial port on startup
3. Select a serial port from the dropdown or device identification list
4. Click "Connect" — the extension negotiates 115200 baud automatically and stores that as the sensor default
5. View live sensor data on the Dashboard tab
6. Configure sentence enable/disable on the Sentences tab
7. View per-message-type data and Hz rates on the Raw Messages tab

## Development

### Building from Source

```bash
git clone https://github.com/vshie/Airmar-WX.git
cd Airmar-WX
docker build -t vshie/blueos-airmar-wx:latest .
```

### Local Testing

```bash
docker-compose up --build
```

Then visit `http://localhost:6436` in your browser.

### GitHub Actions

The CI/CD pipeline requires these GitHub Secrets and Variables:

**Secrets:**
- `DOCKER_USERNAME` — Docker Hub username
- `DOCKER_PASSWORD` — Docker Hub access token (Read & Write)

**Variables:**
- `IMAGE_NAME` — Docker repository name (default: `airmar-wx`)
- `MY_NAME` — Author name
- `MY_EMAIL` — Author email
- `ORG_NAME` — Maintainer organization name
- `ORG_EMAIL` — Maintainer organization email

## License

MIT License - see LICENSE file for details
