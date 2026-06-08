"""Integrated physical path-planning package.

Single home for A->B serpentine geometry, calibration resolution, telemetry and
safety parsing, guarded pulse execution, the continuous-motion controller, and the
unified CLI (modes: preview, calibrate-turn, execute-plan, run, diagnose).

Import edges are one-way (leaves first):
  geometry, calibration, telemetry, safety, checks -> executor -> controller -> cli
Nothing imports controller/cli back.
"""
