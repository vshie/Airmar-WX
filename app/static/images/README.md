# Static images for the extension UI

Drop image assets referenced by `app/static/index.html` in here.

## `airmar-wiring.jpg`

Wiring diagram for the Airmar 300WX → USB-to-RS232 adapter → BlueBoat
power block. Rendered under **Setup → Step 1** as the only content of
that card (per user request the tab is image-led with no prose).

## `blueos-serial-config.png`

Screenshot of BlueOS **Autopilot Firmware → Serial port configuration**
with two serial slots set to:

- Serial 6: `udpin:0.0.0.0:27001` (wind)
- Serial 7: `udpin:0.0.0.0:27002` (GPS/heading)

Rendered under **Setup → Step 2a** next to the copyable `udpin` strings.
If the file is missing, the UI falls back to text guidance automatically.

## `airmar-300wx.png`

Product photo of the Airmar 300WX WeatherStation, used elsewhere in the UI.
