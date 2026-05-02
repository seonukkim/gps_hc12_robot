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
@CMD,104,STOP,0,0*67
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
- `CMD`: command message, including `STOP` and `AUTO`
- `GPS`: rover telemetry
- `STAT`: rover mode / link / RC status
- `ACK`: positive acknowledgement
- `ERR`: negative acknowledgement or parser error

## Safety Expectations

- `STOP` must immediately stop motors on the rover.
- `AUTO` must only be accepted when RC is valid and the RC auto-enable switch is on.
- HC-12 timeout during `AUTO` must force a stop and require explicit re-entry.
