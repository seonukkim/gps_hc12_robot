# Firmware Notes

- Confirm actual OpenRB-150 UART pin mapping before flashing any sketch.
- Confirm logic-level compatibility for HC-12 and GPS UART wiring.
- Treat all motor / ESC tests as wheel-off-ground only.
- Rover firmware owns low-level safety, RC override, and link-loss stopping behavior.

## Integrated Rover Controller

The active rover sketch is:

```text
firmware/openrb_robot_controller/openrb_robot_controller.ino
```

It prints this USB startup marker when the expected firmware is running:

```text
Firmware: openrb_robot_controller station-manual rc-cardinal-remap 2026-05-26
```

Compile with Arduino CLI:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
```

Upload to the connected OpenRB USB serial port:

```bash
arduino-cli upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
```

On Linux/WSL station hosts, the upload port may instead look like
`/dev/ttyACM0`. Confirm the actual port before upload.

Full manual-control bring-up notes are in
[`docs/manual_control.md`](../docs/manual_control.md).

## GPS UART Probe

Use this standalone sketch when validating GPS UART wiring and baudrate:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
```

It does not attach motor outputs. Full procedure and per-variant compile/upload
commands are in [`docs/gps_bringup.md`](../docs/gps_bringup.md).
