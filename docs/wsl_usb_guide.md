# WSL USB Guide

Default station-side serial device is `/dev/ttyACM0`.

## Windows Side

1. List attachable USB devices:

```bash
usbipd list
```

2. Bind the target device if required:

```bash
usbipd bind --busid <BUSID>
```

3. Attach it into WSL:

```bash
usbipd attach --wsl --busid <BUSID>
```

## WSL Side

Verify the device node:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

If the OpenRB appears on a different node, pass it explicitly with `--port`.

## Notes

- Reattach after unplug/replug events if needed.
- Keep your user account in the appropriate serial-access group if permission errors appear.
