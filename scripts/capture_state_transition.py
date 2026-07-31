#!/usr/bin/env python3
"""
Capture serial data from state_transition_analysis firmware and store it in CSV.

The firmware waits for a '\n' character before starting. This script asks the user
for a key press, sends that newline, then records incoming samples.

CSV columns:
- experiment: experiment index (1-based)
- time: time inside each experiment in seconds
- pressure: ADC raw value by default, or calibrated value if slope/offset are set
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import sys
import time
from pathlib import Path

import serial


SAMPLING_HZ = 10_000.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read ESP32 serial data and save experiment/time/pressure to CSV.",
    )
    parser.add_argument("--port", required=True, help="Serial port (example: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV path. Default: state_transition_YYYYmmdd_HHMMSS.csv",
    )
    parser.add_argument(
        "--samples-per-transition",
        type=int,
        default=1000,
        help="Samples printed by firmware per transition (default: 1000)",
    )
    parser.add_argument(
        "--transitions-per-experiment",
        type=int,
        default=2,
        help="Transitions printed per experiment (default: 2)",
    )
    parser.add_argument(
        "--num-experiments",
        type=int,
        default=None,
        help="If set, stop automatically after this many experiments.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Serial read timeout in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--slope",
        type=float,
        default=None,
        help="Optional linear calibration slope: pressure = slope * adc + offset",
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=0.0,
        help="Optional linear calibration offset (default: 0.0)",
    )
    return parser


def default_output_path() -> Path:
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"state_transition_{ts}.csv")


def parse_pressure(adc_value: int, slope: float | None, offset: float) -> float:
    if slope is None:
        return float(adc_value)
    return (slope * float(adc_value)) + offset


def main() -> int:
    args = build_parser().parse_args()

    if args.samples_per_transition <= 0 or args.transitions_per_experiment <= 0:
        print("samples-per-transition and transitions-per-experiment must be > 0", file=sys.stderr)
        return 2

    samples_per_experiment = args.samples_per_transition * args.transitions_per_experiment
    output_path = Path(args.output) if args.output else default_output_path()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Could not open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    with ser:
        # Give the board time to boot/reset when serial opens.
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"Connected to {args.port} at {args.baud} baud")
        print("Press Enter to start the experiment on the ESP32...")
        try:
            input()
        except KeyboardInterrupt:
            print("Cancelled before start")
            return 130

        # Firmware wait_key('\n') unblocks with newline.
        ser.write(b"\n")
        ser.flush()
        print("Start signal sent. Reading samples... (Ctrl+C to stop)")

        experiment_idx = 1
        sample_idx_in_experiment = 0
        total_rows = 0

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="ascii") as f:
            writer = csv.writer(f)
            writer.writerow(["experiment", "time", "pressure"])

            try:
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue

                    line = raw.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    try:
                        adc_value = int(line)
                    except ValueError:
                        # Ignore non-numeric debug lines if they appear.
                        continue

                    pressure_value = parse_pressure(adc_value, args.slope, args.offset)
                    t_s = sample_idx_in_experiment / SAMPLING_HZ

                    writer.writerow([experiment_idx, f"{t_s:.7f}", f"{pressure_value:.8f}"])
                    total_rows += 1
                    sample_idx_in_experiment += 1

                    if sample_idx_in_experiment >= samples_per_experiment:
                        f.flush()
                        print(
                            f"Experiment {experiment_idx} captured "
                            f"({samples_per_experiment} samples)"
                        )
                        experiment_idx += 1
                        sample_idx_in_experiment = 0

                        if args.num_experiments is not None and experiment_idx > args.num_experiments:
                            break

            except KeyboardInterrupt:
                print("Capture interrupted by user")

    print(f"Saved {total_rows} rows to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
