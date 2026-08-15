#!/usr/bin/env python3

import argparse

import serial
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


RRATE_MAP = {
    0x01: "0.2 Hz",
    0x02: "0.5 Hz",
    0x03: "1 Hz",
    0x04: "2 Hz",
    0x05: "5 Hz",
    0x06: "10 Hz",
    0x07: "20 Hz",
    0x08: "50 Hz",
    0x09: "100 Hz",
    0x0B: "200 Hz",
}

BANDWIDTH_MAP = {
    0x00: "256 Hz",
    0x01: "188 Hz",
    0x02: "98 Hz",
    0x03: "42 Hz",
    0x04: "20 Hz",
    0x05: "10 Hz",
    0x06: "5 Hz",
}


def read_register(master, slave_id, address):
    result = master.execute(
        slave_id,
        cst.READ_HOLDING_REGISTERS,
        address,
        1,
    )
    return int(result[0])


def main():
    parser = argparse.ArgumentParser(
        description="Read HWT905-RS485 configuration registers without modifying them."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--slave-id", type=int, default=80)
    parser.add_argument("--serial-timeout", type=float, default=0.5)
    parser.add_argument("--modbus-timeout", type=float, default=1.0)

    args = parser.parse_args()

    print("HWT905-RS485 configuration register reader")
    print(f"port     : {args.port}")
    print(f"baud     : {args.baud}")
    print(f"slave_id : {args.slave_id}")
    print()

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=args.serial_timeout,
    )

    master = modbus_rtu.RtuMaster(ser)
    master.set_timeout(args.modbus_timeout)
    master.set_verbose(False)

    try:
        rrate = read_register(master, args.slave_id, 0x03)
        bandwidth = read_register(master, args.slave_id, 0x1F)
        moddelay = read_register(master, args.slave_id, 0x74)

        print("=== Register values ===")

        rrate_text = RRATE_MAP.get(rrate, "unknown / not mapped")
        print(
            f"RRATE     0x03 : "
            f"raw={rrate} (0x{rrate:04X}) -> {rrate_text}"
        )

        bandwidth_text = BANDWIDTH_MAP.get(
            bandwidth,
            "unknown / not mapped"
        )
        print(
            f"BANDWIDTH 0x1F : "
            f"raw={bandwidth} (0x{bandwidth:04X}) -> {bandwidth_text}"
        )

        print(
            f"MODDELAY  0x74 : "
            f"raw={moddelay} (0x{moddelay:04X}) -> "
            f"{moddelay} us"
        )

    finally:
        try:
            master.close()
        except Exception:
            pass

        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()