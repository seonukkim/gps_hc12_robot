"""Summarize a PPM log captured from firmware/ppm_channel_map_probe.

The probe emits two line types:
  PPMEVT ... CH1_us=.. CH8_us=..   (one per confirmed state change; per-sample)
  PPMSUM ... frames=.. CH1_low=..  (one per second; aggregate window counts)

This tool also accepts raw per-frame lines that carry ``CHn_us=`` (or lowercase
``chn_us=``) pairs, so older rc_channel_probe RCCH logs work too.

It reports, per channel:
  * min / max pulse width
  * how often it was LOW / MID / HIGH
  * whether it changed state during the run
  * the longest HIGH hold (per-sample consecutive, and fully-HIGH 1 s windows)
  * candidate AUTO/MANUAL mode channels (a clean 2-position switch that holds)
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

CHANNEL_COUNT = 8
LOW_MAX_US = 1300
HIGH_MIN_US = 1700
VALID_MIN_US = 800
VALID_MAX_US = 2200

SAMPLE_RE = re.compile(r"\bch(\d+)_us=(\d+)\b", re.IGNORECASE)
SUMMARY_FRAMES_RE = re.compile(r"\bframes=(\d+)\b")
SUMMARY_INVALID_RE = re.compile(r"\binvalid_frames=(\d+)\b")
SUMMARY_FIELD_RE = re.compile(r"\bCH(\d+)_(min|max|low|mid|high)=(\d+)\b")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize a PPM log and suggest stable AUTO/MANUAL mode channels."
    )
    parser.add_argument("paths", nargs="+", help="One or more PPM log files to analyze")
    parser.add_argument(
        "--hold-threshold",
        type=int,
        default=10,
        help="Min consecutive HIGH per-sample frames to call a channel 'holds' (default 10)",
    )
    return parser


def classify(pulse_us: int) -> str:
    if pulse_us < VALID_MIN_US or pulse_us > VALID_MAX_US:
        return "INVALID"
    if pulse_us < LOW_MAX_US:
        return "LOW"
    if pulse_us > HIGH_MIN_US:
        return "HIGH"
    return "MID"


def parse_sample_line(line: str) -> dict[int, int]:
    """Return {channel_index_1based: pulse_us} for any CHn_us pairs on the line."""
    sample: dict[int, int] = {}
    for match in SAMPLE_RE.finditer(line):
        channel = int(match.group(1))
        if 1 <= channel <= CHANNEL_COUNT:
            sample[channel] = int(match.group(2))
    return sample


def parse_summary_line(line: str) -> dict | None:
    """Return aggregate counts for a PPMSUM line, or None if not a summary line."""
    if "PPMSUM" not in line:
        return None
    frames_match = SUMMARY_FRAMES_RE.search(line)
    if not frames_match:
        return None
    record: dict = {
        "frames": int(frames_match.group(1)),
        "invalid": int(SUMMARY_INVALID_RE.search(line).group(1))
        if SUMMARY_INVALID_RE.search(line)
        else 0,
        "channels": {},
    }
    for match in SUMMARY_FIELD_RE.finditer(line):
        channel = int(match.group(1))
        field = match.group(2)
        value = int(match.group(3))
        record["channels"].setdefault(channel, {})[field] = value
    return record


class ChannelStats:
    __slots__ = (
        "sample_min", "sample_max", "low", "mid", "high", "invalid",
        "max_high_run", "_cur_high_run", "fully_high_windows", "max_fully_high_streak",
        "_cur_fully_high_streak",
    )

    def __init__(self) -> None:
        self.sample_min: int | None = None
        self.sample_max: int | None = None
        self.low = 0
        self.mid = 0
        self.high = 0
        self.invalid = 0
        self.max_high_run = 0
        self._cur_high_run = 0
        self.fully_high_windows = 0
        self.max_fully_high_streak = 0
        self._cur_fully_high_streak = 0

    def add_sample(self, pulse_us: int) -> None:
        state = classify(pulse_us)
        if state == "INVALID":
            self.invalid += 1
            self._cur_high_run = 0
            return
        if self.sample_min is None or pulse_us < self.sample_min:
            self.sample_min = pulse_us
        if self.sample_max is None or pulse_us > self.sample_max:
            self.sample_max = pulse_us
        if state == "LOW":
            self.low += 1
            self._cur_high_run = 0
        elif state == "MID":
            self.mid += 1
            self._cur_high_run = 0
        else:
            self.high += 1
            self._cur_high_run += 1
            self.max_high_run = max(self.max_high_run, self._cur_high_run)

    def add_summary_window(self, fields: dict, window_frames: int) -> None:
        low = fields.get("low", 0)
        mid = fields.get("mid", 0)
        high = fields.get("high", 0)
        self.low += low
        self.mid += mid
        self.high += high
        win_min = fields.get("min", 0)
        win_max = fields.get("max", 0)
        if win_min and (self.sample_min is None or win_min < self.sample_min):
            self.sample_min = win_min
        if win_max and (self.sample_max is None or win_max > self.sample_max):
            self.sample_max = win_max
        fully_high = high > 0 and low == 0 and mid == 0 and high == window_frames
        if fully_high:
            self.fully_high_windows += 1
            self._cur_fully_high_streak += 1
            self.max_fully_high_streak = max(
                self.max_fully_high_streak, self._cur_fully_high_streak
            )
        else:
            self._cur_fully_high_streak = 0

    def states_seen(self) -> int:
        return sum(1 for count in (self.low, self.mid, self.high) if count > 0)

    def changed(self) -> bool:
        return self.states_seen() >= 2

    def holds_high(self, hold_threshold: int) -> bool:
        return self.max_high_run >= hold_threshold or self.max_fully_high_streak >= 1

    def is_two_position(self) -> bool:
        return self.low > 0 and self.high > 0 and self.mid == 0


def main() -> int:
    args = build_parser().parse_args()
    stats = {ch: ChannelStats() for ch in range(1, CHANNEL_COUNT + 1)}

    files_read = 0
    sample_lines = 0
    summary_lines = 0

    for raw_path in args.paths:
        path = Path(raw_path)
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                files_read += 1
                for line in handle:
                    summary = parse_summary_line(line)
                    if summary is not None:
                        summary_lines += 1
                        for channel, fields in summary["channels"].items():
                            if channel in stats:
                                stats[channel].add_summary_window(fields, summary["frames"])
                        continue
                    sample = parse_sample_line(line)
                    if sample:
                        sample_lines += 1
                        for channel, pulse in sample.items():
                            stats[channel].add_sample(pulse)
        except FileNotFoundError:
            print(f"Error: file not found: {path}", file=sys.stderr)
        except IsADirectoryError:
            print(f"Error: expected a file, got directory: {path}", file=sys.stderr)
        except OSError as exc:
            print(f"Error: failed to read {path}: {exc}", file=sys.stderr)

    if files_read == 0:
        print("Error: no readable input files.", file=sys.stderr)
        return 1
    if sample_lines == 0 and summary_lines == 0:
        print("Error: no PPM sample (CHn_us) or PPMSUM lines found.", file=sys.stderr)
        return 1

    print(f"Per-sample lines parsed: {sample_lines}")
    print(f"PPMSUM windows parsed:   {summary_lines}")
    print(f"State thresholds: LOW <{LOW_MAX_US}us, MID {LOW_MAX_US}-{HIGH_MIN_US}us, "
          f"HIGH >{HIGH_MIN_US}us; valid {VALID_MIN_US}-{VALID_MAX_US}us")
    print(f"HIGH-hold threshold: {args.hold_threshold} consecutive per-sample frames")
    print()

    header = (
        f"{'CH':>3} {'min':>5} {'max':>5} {'LOW':>7} {'MID':>7} {'HIGH':>7} "
        f"{'INVALID':>7} {'changed':>7} {'maxHIGHrun':>10} {'fullHIwins':>10}"
    )
    print(header)
    print("-" * len(header))

    candidates: list[int] = []
    for channel in range(1, CHANNEL_COUNT + 1):
        cs = stats[channel]
        min_str = str(cs.sample_min) if cs.sample_min is not None else "NA"
        max_str = str(cs.sample_max) if cs.sample_max is not None else "NA"
        print(
            f"{channel:>3} {min_str:>5} {max_str:>5} {cs.low:>7} {cs.mid:>7} {cs.high:>7} "
            f"{cs.invalid:>7} {('yes' if cs.changed() else 'no'):>7} "
            f"{cs.max_high_run:>10} {cs.fully_high_windows:>10}"
        )

        if cs.low > 0 and cs.high > 0 and cs.holds_high(args.hold_threshold):
            candidates.append(channel)

    print()
    print("Channel notes:")
    for channel in range(1, CHANNEL_COUNT + 1):
        cs = stats[channel]
        notes = []
        if not cs.changed():
            notes.append("static (single state) - not a switch")
        if cs.low > 0 and cs.high > 0:
            notes.append("reaches both LOW and HIGH")
        if cs.is_two_position():
            notes.append("clean 2-position (no MID dwell)")
        elif cs.mid > 0 and cs.low > 0 and cs.high > 0:
            notes.append("3-position or analog (has MID dwell)")
        if cs.low > 0 and cs.high > 0 and not cs.holds_high(args.hold_threshold):
            notes.append(f"HIGH does NOT hold (max run {cs.max_high_run}) - momentary/slip?")
        if cs.invalid > 0:
            notes.append(f"{cs.invalid} invalid/out-of-range samples - possible channel slip")
        if notes:
            print(f"  CH{channel}: " + "; ".join(notes))

    print()
    if candidates:
        best = ", ".join(f"CH{ch}" for ch in candidates)
        print(f"Candidate stable AUTO/MANUAL mode channel(s): {best}")
        print("These reach both LOW and HIGH and hold HIGH. Verify with a slow switch flip.")
    else:
        print("Candidate stable AUTO/MANUAL mode channel(s): NONE")
        print("No channel held HIGH long enough. Do not enable path following.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
