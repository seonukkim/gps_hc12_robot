# GPS Serial3 Test Result

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
