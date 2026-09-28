#!/usr/bin/env python3

from pymodbus.client import ModbusSerialClient

PORT = "/dev/ttyUSB0"
BAUD = 230400
SLAVE_ID = 80

client = ModbusSerialClient(
    port=PORT,
    baudrate=BAUD,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=1
)

print("connecting...")
print(client.connect())

rr = client.read_holding_registers(
    address=0x34,
    count=12,
    slave=SLAVE_ID
)

print(rr)

if rr.isError():
    print("Modbus error")
else:
    print(rr.registers)

client.close()
