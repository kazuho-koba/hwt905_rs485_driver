#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import serial
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

START_REGISTER = 0x34
REGISTER_COUNT = 12
IDX_GY = 4       # 0x38: Y-axis gyro
IDX_PITCH = 10   # 0x3E: Pitch


def signed16(v):
    return v - 65536 if v > 32767 else v


def gyro_dps(raw):
    return raw / 32768.0 * 2000.0


def angle_deg(raw):
    return raw / 32768.0 * 180.0


def percentile(values, q):
    xs = sorted(values)
    if not xs:
        return float("nan")
    if len(xs) == 1:
        return xs[0]
    p = (len(xs) - 1) * q
    lo = int(math.floor(p))
    hi = int(math.ceil(p))
    if lo == hi:
        return xs[lo]
    a = p - lo
    return xs[lo] * (1.0 - a) + xs[hi] * a


def main():
    ap = argparse.ArgumentParser(
        description="Check HWT905-RS485 effective internal register update rate."
    )
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--slave-id", type=int, default=80)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--warmup", type=int, default=30)
    ap.add_argument(
        "--min-gyro-dps",
        type=float,
        default=10.0,
        help="Pitch-motion threshold using |GY| [deg/s]",
    )
    ap.add_argument("--serial-timeout", type=float, default=0.5)
    ap.add_argument("--modbus-timeout", type=float, default=1.0)
    ap.add_argument("--output", default="")
    args = ap.parse_args()

    if args.output:
        out = Path(args.output)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = (
            Path(__file__).resolve().parent
            / "results"
            / f"hwt905_internal_update_{stamp}.csv"
        )
    out.parent.mkdir(parents=True, exist_ok=True)

    print("HWT905-RS485 internal update-rate check")
    print(f"port          : {args.port}")
    print(f"baud          : {args.baud}")
    print(f"slave_id      : {args.slave_id}")
    print(f"duration      : {args.duration:.1f} s")
    print(f"min |GY|      : {args.min_gyro_dps:.1f} deg/s")
    print(f"output        : {out}")
    print()
    print("Continuously rock the UGV around the Y axis (pitch) during acquisition.")
    print("No device settings are changed.")
    print()

    ser = serial.Serial(
        args.port,
        args.baud,
        timeout=args.serial_timeout,
    )
    master = modbus_rtu.RtuMaster(ser)
    master.set_timeout(args.modbus_timeout)
    master.set_verbose(False)

    rows = []
    errors = 0

    try:
        for _ in range(args.warmup):
            master.execute(
                args.slave_id,
                cst.READ_HOLDING_REGISTERS,
                START_REGISTER,
                REGISTER_COUNT,
            )

        print("Recording started...")
        t0_ns = time.perf_counter_ns()
        deadline_ns = t0_ns + int(args.duration * 1e9)

        while time.perf_counter_ns() < deadline_ns:
            req_ns = time.perf_counter_ns()

            try:
                reg = master.execute(
                    args.slave_id,
                    cst.READ_HOLDING_REGISTERS,
                    START_REGISTER,
                    REGISTER_COUNT,
                )
            except Exception as e:
                errors += 1
                print(f"Read error #{errors}: {e}")
                continue

            recv_ns = time.perf_counter_ns()
            v = [signed16(x) for x in reg]

            gy_raw = v[IDX_GY]
            pitch_raw = v[IDX_PITCH]

            rows.append({
                "sample": len(rows),
                "t_s": (recv_ns - t0_ns) / 1e9,
                "transaction_ms": (recv_ns - req_ns) / 1e6,
                "gyro_y_raw": gy_raw,
                "gyro_y_dps": gyro_dps(gy_raw),
                "pitch_raw": pitch_raw,
                "pitch_deg": angle_deg(pitch_raw),
            })

        print("Recording finished.")

    finally:
        try:
            master.close()
        except Exception:
            pass
        if ser.is_open:
            ser.close()

    if len(rows) < 2:
        raise RuntimeError("Not enough samples were collected.")

    rows[0]["gyro_changed"] = ""
    rows[0]["pitch_changed"] = ""
    rows[0]["pair_changed"] = ""

    active_pairs = 0
    active_pair_changes = 0
    active_gyro_changes = 0
    active_pitch_changes = 0
    active_dts = []

    pair_runs = []
    run_len = 1

    for prev, cur in zip(rows[:-1], rows[1:]):
        gy_changed = cur["gyro_y_raw"] != prev["gyro_y_raw"]
        pitch_changed = cur["pitch_raw"] != prev["pitch_raw"]
        pair_changed = gy_changed or pitch_changed

        cur["gyro_changed"] = int(gy_changed)
        cur["pitch_changed"] = int(pitch_changed)
        cur["pair_changed"] = int(pair_changed)

        if pair_changed:
            pair_runs.append(run_len)
            run_len = 1
        else:
            run_len += 1

        active = (
            abs(prev["gyro_y_dps"]) >= args.min_gyro_dps
            or abs(cur["gyro_y_dps"]) >= args.min_gyro_dps
        )

        if active:
            active_pairs += 1
            active_dts.append(cur["t_s"] - prev["t_s"])
            active_pair_changes += int(pair_changed)
            active_gyro_changes += int(gy_changed)
            active_pitch_changes += int(pitch_changed)

    pair_runs.append(run_len)

    duration = rows[-1]["t_s"] - rows[0]["t_s"]
    polling_hz = (len(rows) - 1) / duration

    tx = [r["transaction_ms"] for r in rows]
    tx_mean = statistics.fmean(tx)
    tx_median = statistics.median(tx)
    tx_p95 = percentile(tx, 0.95)
    tx_p99 = percentile(tx, 0.99)

    all_pair_changes = sum(
        int(r["pair_changed"])
        for r in rows[1:]
    )
    all_gyro_changes = sum(
        int(r["gyro_changed"])
        for r in rows[1:]
    )
    all_pitch_changes = sum(
        int(r["pitch_changed"])
        for r in rows[1:]
    )

    print()
    print("=" * 72)
    print("=== Acquisition summary ===")
    print(f"valid samples          : {len(rows)}")
    print(f"errors                 : {errors}")
    print(f"duration               : {duration:.3f} s")
    print(f"polling rate           : {polling_hz:.3f} Hz")
    print(
        "transaction [ms]      : "
        f"mean={tx_mean:.3f}, median={tx_median:.3f}, "
        f"p95={tx_p95:.3f}, p99={tx_p99:.3f}, max={max(tx):.3f}"
    )

    print()
    print("=== Full-record value changes ===")
    print(
        f"GY changes             : {all_gyro_changes} "
        f"({all_gyro_changes / duration:.3f}/s)"
    )
    print(
        f"Pitch changes          : {all_pitch_changes} "
        f"({all_pitch_changes / duration:.3f}/s)"
    )
    print(
        f"GY+Pitch changes       : {all_pair_changes} "
        f"({all_pair_changes / duration:.3f}/s)"
    )

    print()
    print("=== Pitch-motion active region ===")
    print(f"active adjacent pairs  : {active_pairs}")

    if active_pairs > 0:
        mean_active_dt = statistics.fmean(active_dts)
        active_poll_hz = 1.0 / mean_active_dt
        pair_ratio = active_pair_changes / active_pairs
        gyro_ratio = active_gyro_changes / active_pairs
        pitch_ratio = active_pitch_changes / active_pairs
        heuristic_hz = pair_ratio * active_poll_hz

        print(f"active polling rate    : {active_poll_hz:.3f} Hz")
        print(f"GY change ratio        : {gyro_ratio * 100.0:.2f}%")
        print(f"Pitch change ratio     : {pitch_ratio * 100.0:.2f}%")
        print(f"GY+Pitch change ratio  : {pair_ratio * 100.0:.2f}%")

        print()
        print("=== Automatic interpretation ===")

        if active_pairs < 100:
            print(
                "Too few motion-active samples. Repeat while continuously "
                "rocking the UGV around pitch."
            )
        elif pair_ratio >= 0.98:
            print(
                f"Almost every read returned a different GY+Pitch state. "
                f"Internal refresh is therefore at least about "
                f"{active_poll_hz:.1f} Hz."
            )
            print(
                "The true rate may be higher (for example 200 Hz); this test "
                "cannot distinguish rates above the Modbus polling rate."
            )
        elif pair_ratio >= 0.90:
            print(
                f"Most reads returned a new state. Internal refresh is close "
                f"to the polling rate ({active_poll_hz:.1f} Hz) or higher."
            )
            print(
                f"A simple transition-ratio estimate is about "
                f"{heuristic_hz:.1f} Hz."
            )
        else:
            print(
                f"Repeated states are frequent enough to indicate an internal "
                f"refresh rate below the polling rate."
            )
            print(
                f"Simple transition-ratio estimate: about "
                f"{heuristic_hz:.1f} Hz."
            )
            print(
                "If this is close to 100 Hz while polling is around 130-135 Hz, "
                "that is strong evidence for about 100 Hz register refresh."
            )

    print()
    print("=== Identical GY+Pitch run lengths ===")
    hist = Counter(pair_runs)
    for length in sorted(hist)[:10]:
        print(f"run length {length:2d}: {hist[length]} occurrence(s)")

    fields = [
        "sample",
        "t_s",
        "transaction_ms",
        "gyro_y_raw",
        "gyro_y_dps",
        "pitch_raw",
        "pitch_deg",
        "gyro_changed",
        "pitch_changed",
        "pair_changed",
    ]

    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in fields})

    print()
    print(f"CSV saved: {out}")


if __name__ == "__main__":
    main()
