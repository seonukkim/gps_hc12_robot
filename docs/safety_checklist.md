# Safety Checklist

- default station behavior is heartbeat plus `STOP`
- no `AUTO` on startup
- RC override takes priority over station `AUTO`
- link timeout during rover `AUTO` forces motor stop
- reconnection does not auto-resume `AUTO`
- wheel-off-ground only for motor / ESC bench testing
- confirm UART voltage compatibility before wiring
- confirm OpenRB UART pin mapping before final firmware upload
