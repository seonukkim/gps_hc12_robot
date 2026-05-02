# Architecture

## Ground Station

- Platform: WSL2 Ubuntu 24.04 or Jetson
- Stack: Python, `uv`, ROS2 Jazzy
- Radio link: HC-12 over USB serial, default `/dev/ttyACM0` at `9600`
- Responsibilities:
  - high-level mission logic
  - coverage path planning
  - GPS telemetry logging
  - ROS2 bridging and orchestration
  - operator-facing diagnostics

Ground-station defaults are intentionally conservative: heartbeat plus `STOP` only unless the operator explicitly changes behavior.

## Rover

- Controller: OpenRB-150
- Peripherals:
  - HC-12 UART
  - GPS UART
  - RC receiver PPM
  - ESC / motor outputs
- Responsibilities:
  - low-level safety ownership
  - RC/manual priority
  - link-loss handling
  - motion actuation
  - telemetry generation

## Safety Boundary

- OpenRB owns the final safety decision for motion.
- RC manual override must always win over station-side `AUTO`.
- HC-12 timeout during `AUTO` must force `STOP`.
- Reconnection must not automatically resume `AUTO`.
- Station startup must not send live drive commands.

## Protocol Boundary

The serial protocol is line-based:

```text
@TYPE,SEQ,PAYLOAD*CS
```

- `TYPE`: short message type such as `HB`, `CMD`, `GPS`, `STAT`, `ACK`
- `SEQ`: integer sequence number
- `PAYLOAD`: comma-separated fields
- `CS`: XOR checksum of the body text without `@` and without `*CS`

Core protocol and planner code remains independent from ROS2 so it can be reused from plain Python tools and firmware test harnesses.
