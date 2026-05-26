# Protocol

The current rover/station link uses a simple ASCII line protocol over HC-12
serial. Keep this protocol stable before adding ROS2 runtime behavior or
micro-ROS.

## Frame Format

```text
@TYPE,SEQ,PAYLOAD*CS\n
```

Checksum `CS` is the uppercase two-digit XOR checksum over the body string:

```text
TYPE,SEQ,PAYLOAD
```

Do not include `@`, `*`, `CS`, `\r`, or `\n` in the checksum input.

## Validation Rules

Software decoders must reject malformed frames:

- frame must be ASCII
- frame must start with `@`
- frame must contain `*`
- checksum must be two hex digits
- checksum must match the XOR of the body
- body must contain `TYPE`, `SEQ`, and `PAYLOAD`
- `TYPE` must be an uppercase ASCII token
- `SEQ` must parse as an integer
- payload must not contain newline characters

Python implementation:

```text
gps_coverage_core/protocol.py
```

Rover firmware implementation:

```text
firmware/openrb_robot_controller/openrb_robot_controller.ino
```

## Message Types

| Type | Direction | Current Meaning |
|---|---|---|
| `HB` | station -> rover | heartbeat, payload `STATION` |
| `CMD` | station -> rover | command payload: `STOP`, `MANUAL`, `AUTO`, or `START` |
| `GPS` | rover -> station | GPS telemetry payload |
| `STAT` | rover -> station | rover mode, RC state, link state, reference sequence |
| `ACK` | rover -> station | accepted heartbeat or command |
| `ERR` | rover -> station | rejected command, unsupported type, or parser error |

## Current Commands

### Heartbeat

```text
@HB,SEQ,STATION*CS
```

Heartbeat updates station link freshness. It must not imply motion.

### STOP

```text
@CMD,SEQ,STOP,0,0*CS
```

STOP must neutralize motors immediately, clear station manual state, clear
station auto command state, and take precedence over any mission command.

### Station Manual

```text
@CMD,SEQ,MANUAL,steer,throttle,deadman,estop*CS
```

Fields:

- `steer`: normalized `-1.0..1.0`
- `throttle`: normalized `-1.0..1.0`
- `deadman`: `0` or `1`
- `estop`: `0` or `1`

Rover requirements:

- drive only when manual frames are fresh
- drive only when `deadman=1`
- stop when `estop=1`
- stop when manual frames become stale
- cap station manual output to `STATION_MANUAL_MAX_OUTPUT = 0.25f`

Station keyboard manual starts disarmed and sends heartbeat plus STOP until the
operator explicitly arms it.

### AUTO / START

```text
@CMD,SEQ,AUTO,left,right*CS
@CMD,SEQ,START,left,right*CS
```

Fields:

- `left`: normalized `-1.0..1.0`
- `right`: normalized `-1.0..1.0`

Current status:

- the rover firmware can parse and gate this command
- station startup must not send it
- path generation must not send it
- early tests are wheel-off-ground only
- live autonomous execution is not implemented

Rover accepts AUTO/START only when:

- RC channels are valid
- RC mode switch is in auto-ready state
- station link is fresh
- station E-stop is not active

HC-12 timeout during AUTO must stop the rover and require explicit re-entry.

## Rover Telemetry

### STAT

Current firmware payload shape:

```text
MODE,RC_OK|RC_BAD,LINK_OK|LINK_LOST,ref_seq
```

Example:

```text
@STAT,521,AUTO_READY,RC_OK,LINK_OK,103*CS
```

`STAT` is status telemetry only. It should not be used as an implicit command.

### GPS

Current firmware emits key/value payloads:

```text
fix=1,lat=35.123456,lon=129.123456,sats=8,hdop=1.14,age_ms=120
```

Known mismatch:

- firmware emits key/value GPS payloads
- `gps_coverage_core.telemetry.GPSTelemetry.from_payload()` currently expects
  positional fields: `lat,lon,alt,sats,hdop,fix_valid`

Before relying on station-side live GPS telemetry, choose one schema and update
firmware, Python parser, docs, and tests together.

## Examples

The checksum values below are examples; regenerate them with
`gps_coverage_core.protocol.encode_frame()` when adding tests.

```text
@HB,102,STATION*CS
@CMD,103,STOP,0,0*CS
@CMD,104,MANUAL,0.1,0.0,1,0*CS
@CMD,105,AUTO,0.25,-0.03*CS
@GPS,520,fix=1,lat=35.123456,lon=129.123456,sats=8,hdop=1.14,age_ms=120*CS
@STAT,521,AUTO_READY,RC_OK,LINK_OK,103*CS
@ACK,103,OK*CS
@ERR,104,AUTO_REJECTED*CS
```

## Protocol Rules For Next Milestones

- Add no new motion command until STOP, heartbeat, status, and GPS parsing are
  verified end-to-end.
- Keep default station behavior dry-run or heartbeat plus STOP.
- Any path plan approval must be explicit and logged.
- Command sequence numbers should be logged with telemetry and operator action.
- Do not require ROS2 or micro-ROS for basic protocol validation.
