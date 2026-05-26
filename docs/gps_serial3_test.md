# GPS Serial3 Test Result

> Historical result: this page records a prior successful `Serial3` / `D13` /
> `9600` GPS test. On 2026-05-26, a fresh GPS-only test on the same assumed
> input reported `chars_1s=0 total_chars=0 tinygps_chars=0`, so the current GPS
> UART path must be revalidated with `docs/gps_bringup.md`. The follow-up probe
> confirmed the current central OpenRB connector as `Serial2` at `9600`, not
> `Serial3` D13/D14, and reached `fix=true`.

## Confirmed wiring

GPS module pin order used in test:

- VCC / UCC
- GND
- TX
- RX
- PPS

OpenRB-150 wiring:

| GPS pin | OpenRB-150 pin |
|---|---|
| VCC / UCC | +5V |
| GND | GND |
| TX | D13 / RX |
| RX | not connected for read-only test |
| PPS | not connected |

## Confirmed firmware mapping

- GPS UART: `Serial3`
- GPS baudrate: `9600`
- USB debug baudrate: `115200`

## Success criterion

The test succeeded when `chars_1s > 0` and later `status=FIX`.

Observed result:

- `chars_1s`: approximately 380-397
- `status`: FIX
- `sats`: 3
- `hdop`: approximately 3.24

## Notes

- `NO_FIX` indoors or near a window can be normal.
- `chars_1s > 0` means GPS serial wiring is correct.
- For navigation, prefer outdoor testing with `sats >= 4` and lower HDOP.
