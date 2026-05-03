# Architecture

## Status Note

This is the target architecture and current boundary model for the pre-ROS2 Python/OpenRB prototype. Python core protocol and planning code are implemented independently from ROS2. ROS2 Jazzy orchestration is planned later, and station-side HC-12 end-to-end operation should be described as pending until the station hardware and link test are confirmed.

## Ground Station

- Platform: WSL2 Ubuntu 24.04 or Jetson
- Current stack: Python and `uv`
- Planned stack: ROS2 Jazzy integration on top of the ROS-independent core modules
- Intended radio link: HC-12 over USB serial, default `/dev/ttyACM0` at `9600`; station-side end-to-end confirmation is pending
- Current responsibilities:
  - high-level mission logic for safe tests and offline or mock missions
  - coverage path planning
  - GPS telemetry logging
  - operator-facing diagnostics
- Planned responsibilities:
  - ROS2 bridging and orchestration

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
