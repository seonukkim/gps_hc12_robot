# Field Test Log

Use this file as the project-level index for bench and field tests. Detailed raw
logs should remain in `data/` or another run-specific output directory.

All motor tests are wheel-off-ground unless a later entry explicitly documents a
safe ground-contact setup.

## Log Template

```text
Date:
Location:
Operator:
Firmware marker:
Station command:
Serial port:
GPS fix:
RC mode:
Purpose:
Procedure:
Observed result:
STOP behavior:
Artifacts/logs:
Next action:
```

## 2026-05-26: Integrated Firmware Upload And Manual Direction Work

Firmware marker:

```text
openrb_robot_controller station-manual rc-cardinal-remap 2026-05-26
```

Station command:

```text
not a station motion test; OpenRB USB firmware upload and USBDBG check
```

Serial port:

```text
/dev/cu.usbmodem12101
```

Purpose:

- Put the repository integrated rover firmware onto the OpenRB.
- Preserve manual control and station manual protocol behavior.
- Correct RC manual axis mapping so straight up/down are intended to be
  forward/reverse.

Observed result:

- Arduino compile succeeded.
- OpenRB upload succeeded.
- Flash verify succeeded.
- USBDBG reported neutral command state:
  - `manual_steer_cmd=0.000`
  - `manual_throttle_cmd=0.000`
  - `left_cmd=0.000`
  - `right_cmd=0.000`

STOP behavior:

- Neutral startup confirmed through USBDBG.
- Full wheel-off-ground direction validation still needs operator observation.

Artifacts/logs:

- firmware source: `firmware/openrb_robot_controller/openrb_robot_controller.ino`
- procedure docs: `docs/manual_control.md`

Next action:

- Wheel-off-ground cardinal direction validation:
  - straight up should produce forward
  - straight down should produce reverse
  - straight left/right should steer without large throttle bias

## Known Manual Direction Attempts

These are recorded to prevent repeating the same fixes:

| Attempt | Observed Result | Status |
|---|---|---|
| old unknown board firmware | manual control existed, but source was not in repo | replaced by integrated repo firmware |
| first 45-degree remap | left/right behaved like forward/reverse | rejected |
| direct CH1/CH2 map | straight up/down did not align with forward/reverse | rejected |
| direct CH2 inversion | upper-left became forward and lower-right became reverse | rejected |
| current cardinal remap | intended to rotate raw diagonal axes into straight up/down/left/right | uploaded; needs wheel-off-ground direction validation |

## 2026-05-03: Historical Verified Status From Existing Docs

Source:

- `docs/current_hardware_status.md`
- `docs/project_notes/repo_audit.md`

Confirmed at that time:

- OpenRB USB debug working.
- RC receiver PPM input working.
- RC manual mode working.
- Failsafe STOP behavior working.
- GPS module communication working.
- GPS FIX confirmed.

Pending at that time:

- station-side HC-12 USB confirmation
- end-to-end station HC-12 link test
- GPS telemetry schema reconciliation
