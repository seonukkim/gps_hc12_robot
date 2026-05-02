# Protocol

Frame format:

```text
@TYPE,SEQ,PAYLOAD*CS\n
```

`CS` is the XOR checksum over the body string:

```text
TYPE,SEQ,PAYLOAD
```

Do not include `@` or `*CS` in the checksum input.

## Examples

```text
@HB,102,STATION*63
@CMD,103,AUTO,0.25,-0.03*5E
@CMD,104,MANUAL,0.1,0.0,1,0*65
@CMD,105,STOP,0,0*66
@GPS,520,35.123456,129.123456,48.2,8,1.4,1*51
@STAT,521,AUTO_RUNNING,RC_OK,LINK_OK,103*3C
@ACK,103,OK*7F
```

## Validation Rules

- frame must start with `@`
- frame must include `*`
- checksum must match
- `SEQ` must parse as an integer
- malformed input must raise an error in software parsers

## Message Types

- `HB`: station heartbeat
- `CMD`: command message, including `STOP`, `AUTO`, and `MANUAL`
- `GPS`: rover telemetry
- `STAT`: rover mode / link / RC status
- `ACK`: positive acknowledgement
- `ERR`: negative acknowledgement or parser error

## Command Payloads

- `CMD,AUTO,left,right`
  - explicit station-side auto start/update command
  - `left` and `right` are normalized `-1.0..1.0`
- `CMD,MANUAL,steer,throttle,deadman,estop`
  - `SEQ` is already carried by the frame header and is not duplicated in the payload
  - `steer` and `throttle` are normalized `-1.0..1.0`
  - `deadman` and `estop` are boolean `0/1`
- `CMD,STOP,0,0`
  - immediate stop request; the extra zero fields are kept for compatibility with existing tools

## Safety Expectations

- `STOP` must immediately stop motors on the rover.
- `MANUAL` must only drive when fresh manual frames continue to arrive and `deadman=1`.
- Station manual timeout is `<= 500 ms`; stale manual frames must return the rover to stop.
- First station-manual tests are limited to `0.25` output magnitude or lower.
- `AUTO` must only be accepted when RC is valid, the RC auto-enable switch is on, and the station sends an explicit `AUTO`/start command.
- CH5 high by itself must not enter `AUTO_RUNNING`; it only places the rover in `AUTO_READY`.
- HC-12 timeout during `AUTO` must force a stop and require explicit re-entry.
