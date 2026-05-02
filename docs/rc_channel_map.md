# RC Channel Map

Proposed mapping. Final RC assignment must be confirmed on the actual transmitter / receiver pair.

- `CH1`: steering / yaw
- `CH2`: throttle
- `CH3`: arm / disarm
- `CH4`: auto-enable switch
- `CH5`: auxiliary mode
- `CH6`: spare
- `CH7`: spare
- `CH8`: spare

Implementation note:

- Manual mode must override station `AUTO`.
- The rover controller should only accept station `AUTO` when RC is valid and the configured auto-enable switch is active.
