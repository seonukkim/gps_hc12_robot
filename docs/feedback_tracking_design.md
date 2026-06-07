# Feedback Tracking Design For Side-Tool Paths

This is a no-motion design note. It does not enable physical path following,
does not change firmware motor gates, and does not authorize rover motion.

## Purpose

The side-mounted tool planner creates an offline A/B-bounded path preview. A
future physical tracker would need to follow that preview while continuously
correcting position and heading from GPS and BMI160. That future tracker must
remain separate from the current offline preview until a dedicated safety review
approves any motor-output path.

## Coordinate Frame

1. Convert operator-selected GPS A/B points into a local metric frame.
2. Local x points from A to B.
3. Local y points toward `workspace_side`.
4. Convert live GPS lat/lon into the same A/B frame.
5. Compare the rover chassis center against the active preview segment.

The tool footprint is offset from chassis center, so feedback must track both:

- chassis centerline pose;
- side-mounted tool footprint position.

## Heading Sources

BMI160 yaw is relative. It is useful for short-term heading propagation, but it
is not an absolute compass heading.

Future heading fusion should use:

- BMI160 relative yaw for short-term rotation changes;
- GPS course-over-ground when movement exceeds the configured displacement
  threshold;
- GPS course updates to correct BMI160 yaw drift while moving.

If the rover is stationary or moving too little, GPS course is not reliable and
heading correction must be blocked.

## Segment Tracking

For each preview segment:

1. project current GPS-derived chassis position onto the segment;
2. compute cross-track error;
3. compute along-track progress;
4. advance segment state when along-track progress reaches the segment end;
5. handle forward and reverse lanes separately;
6. keep chassis heading distinct from actual travel direction.

Side-step transitions remain semantic preview segments:

- `rotate_90`
- `reverse_offset`
- `rotate_back`

They are not motor commands in the current project state.

## Simulation Outputs

Offline simulation may produce:

- `cross_track_error_m`
- `along_track_progress_m`
- `heading_error_deg`
- `virtual_desired_forward_cmd`
- `virtual_desired_turn_cmd`

These fields are diagnostic only. They must remain named `virtual_*` and must
not be streamed to rover firmware as motor commands.

## Required Gates Before Any Future Motion

Before any physical route execution can be discussed, a separate safety review
must confirm:

- GPS OK outdoors;
- BMI160 OK and plausible;
- RC/manual OK;
- STOP / E-stop behavior verified;
- no-motion logs still show `physical_block_reason=COMPILE_GATE_OFF`;
- no-motion logs still show `physical_output_active=false`;
- no-motion logs still show `final_left_cmd=0.000` and `final_right_cmd=0.000`;
- explicit human approval exists for a guarded crawl discussion.

Physical path following remains prohibited until those gates are reviewed and
new firmware changes are explicitly requested.

## Replacing Immediate Hand-Carry Route Validation

The immediate checklist can avoid hand-carry route validation by using:

1. offline geometry validation with `boundary_violation_count=0`;
2. offline tracking simulation from `side_tool_path.csv`;
3. integrated no-motion target logging;
4. actuator-free or wheels-off checks only if a future safety plan requires
   them;
5. guarded crawl only after separate explicit approval.

This replacement does not weaken motor safety gates. It only reduces outdoor
iteration count before future no-motion validation.
