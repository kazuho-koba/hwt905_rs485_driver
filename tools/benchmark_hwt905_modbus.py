#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial
from modbus_tk import modbus_rtu
import modbus_tk.defines as cst


def percentile(values, q):
    if not values:
        return float("nan")
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs[lo]
    frac = pos - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def summarize(values):
    if not values:
        return {k: float("nan") for k in ("mean", "median", "p95", "p99", "min", "max")}
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "min": min(values),
        "max": max(values),
    }


def signed16(x):
    return x - 65536 if x > 32767 else x


def mimic_processing(reg):
    v = [signed16(x) for x in reg]

    if len(v) >= 3:
        _accel = [v[i] / 32768.0 * 16.0 * 9.8 for i in range(0, 3)]

    if len(v) >= 6:
        _gyro = [
            v[i] / 32768.0 * 2000.0 * math.pi / 180.0
            for i in range(3, 6)
        ]

    if len(v) >= 9:
        _mag = [float(x) for x in v[6:9]]

    if len(v) >= 12:
        angle_degree = [v[i] / 32768.0 * 180.0 for i in range(9, 12)]
        roll, pitch, yaw = [x * math.pi / 180.0 for x in angle_degree]
        yaw -= math.pi / 2.0

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        _quat = (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )


def parse_counts(text):
    values = [int(x.strip()) for x in text.split(",") if x.strip()]
    if not values or any(v <= 0 for v in values):
        raise argparse.ArgumentTypeError("counts must be positive integers")
    return values


def run_case(master, slave_id, address, register_count, iterations, warmup, target_hz):
    period_s = 1.0 / target_hz if target_hz > 0.0 else 0.0

    for _ in range(warmup):
        master.execute(
            slave_id,
            cst.READ_HOLDING_REGISTERS,
            address,
            register_count,
        )

    rows = []
    prev_start_ns = None

    for i in range(iterations):
        loop_start_ns = time.perf_counter_ns()

        period_ms = ""
        if prev_start_ns is not None:
            period_ms = (loop_start_ns - prev_start_ns) / 1e6
        prev_start_ns = loop_start_ns

        tx_start_ns = time.perf_counter_ns()

        try:
            reg = master.execute(
                slave_id,
                cst.READ_HOLDING_REGISTERS,
                address,
                register_count,
            )
        except Exception as exc:
            rows.append({
                "iteration": i,
                "register_count": register_count,
                "transaction_ms": "",
                "processing_ms": "",
                "work_ms": "",
                "sleep_requested_ms": "",
                "period_ms": period_ms,
                "error": repr(exc),
            })
            continue

        tx_end_ns = time.perf_counter_ns()

        proc_start_ns = tx_end_ns
        mimic_processing(reg)
        proc_end_ns = time.perf_counter_ns()

        transaction_ms = (tx_end_ns - tx_start_ns) / 1e6
        processing_ms = (proc_end_ns - proc_start_ns) / 1e6
        work_s = (proc_end_ns - loop_start_ns) / 1e9
        work_ms = work_s * 1e3

        sleep_requested_s = 0.0
        if period_s > 0.0:
            sleep_requested_s = max(0.0, period_s - work_s)
            if sleep_requested_s > 0.0:
                time.sleep(sleep_requested_s)

        rows.append({
            "iteration": i,
            "register_count": register_count,
            "transaction_ms": transaction_ms,
            "processing_ms": processing_ms,
            "work_ms": work_ms,
            "sleep_requested_ms": sleep_requested_s * 1e3,
            "period_ms": period_ms,
            "error": "",
        })

    return rows


def print_summary(rows, register_count, target_hz):
    valid = [r for r in rows if not r["error"]]
    tx = [float(r["transaction_ms"]) for r in valid]
    proc = [float(r["processing_ms"]) for r in valid]
    work = [float(r["work_ms"]) for r in valid]
    periods = [float(r["period_ms"]) for r in valid if r["period_ms"] != ""]
    sleeps = [float(r["sleep_requested_ms"]) for r in valid]

    print()
    print("=" * 72)
    print(f"register_count : {register_count}")
    print(f"target_hz      : {target_hz if target_hz > 0 else 'unlimited'}")
    print(f"valid samples  : {len(valid)}")
    print(f"errors         : {len(rows) - len(valid)}")

    for name, values in (
        ("transaction", tx),
        ("processing ", proc),
        ("work       ", work),
        ("period     ", periods),
        ("sleep req. ", sleeps),
    ):
        s = summarize(values)
        print(
            f"{name} [ms] : "
            f"mean={s['mean']:.3f}, median={s['median']:.3f}, "
            f"p95={s['p95']:.3f}, p99={s['p99']:.3f}, "
            f"min={s['min']:.3f}, max={s['max']:.3f}"
        )

    if periods:
        mean_period_ms = statistics.fmean(periods)
        print(f"achieved_hz    : {1000.0 / mean_period_ms:.3f}")

    if work:
        mean_work_ms = statistics.fmean(work)
        print(f"mean-work ceiling (rough) : {1000.0 / mean_work_ms:.3f} Hz")
        over_10ms = sum(x > 10.0 for x in work)
        print(
            f"work > 10 ms   : {over_10ms}/{len(work)} "
            f"({100.0 * over_10ms / len(work):.1f}%)"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--slave-id", type=int, default=80)
    parser.add_argument("--address", type=int, default=52)
    parser.add_argument("--counts", type=parse_counts, default=parse_counts("12"))
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--target-hz", type=float, default=100.0,
                        help="0 means run as fast as possible")
    parser.add_argument("--serial-timeout", type=float, default=0.5)
    parser.add_argument("--modbus-timeout", type=float, default=1.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    if args.output:
        output_path = Path(args.output)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = (
            Path(__file__).resolve().parent
            / "results"
            / f"hwt905_modbus_timing_{timestamp}.csv"
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("HWT905 Modbus timing benchmark")
    print(f"port            : {args.port}")
    print(f"baud            : {args.baud}")
    print(f"slave_id        : {args.slave_id}")
    print(f"start address   : {args.address}")
    print(f"register counts : {args.counts}")
    print(f"iterations      : {args.iterations}")
    print(f"warmup          : {args.warmup}")
    print(f"target_hz       : {args.target_hz if args.target_hz > 0 else 'unlimited'}")
    print(f"output          : {output_path}")

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=args.serial_timeout,
    )
    master = modbus_rtu.RtuMaster(ser)
    master.set_timeout(args.modbus_timeout)
    master.set_verbose(False)

    all_rows = []

    try:
        for count in args.counts:
            print()
            print(f"Running register_count={count} ...")
            rows = run_case(
                master,
                args.slave_id,
                args.address,
                count,
                args.iterations,
                args.warmup,
                args.target_hz,
            )
            all_rows.extend(rows)
            print_summary(rows, count, args.target_hz)
    finally:
        try:
            master.close()
        except Exception:
            pass
        if ser.is_open:
            ser.close()

    fieldnames = [
        "iteration",
        "register_count",
        "transaction_ms",
        "processing_ms",
        "work_ms",
        "sleep_requested_ms",
        "period_ms",
        "error",
    ]

    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)

    print()
    print(f"CSV saved: {output_path}")


if __name__ == "__main__":
    main()
