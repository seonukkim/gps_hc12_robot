# Repository Guidance

- Use `uv` for dependency and task execution.
- Preserve safe station defaults: heartbeat and `STOP` only unless explicitly changed.
- Default USB serial device is `/dev/ttyACM0`; expose `--port` in serial tools.
- Do not introduce station-side startup behavior that sends live motor-driving `AUTO` commands.
- Keep core protocol and planning modules independent from ROS2.
- Treat rover motor testing as wheel-off-ground only.
