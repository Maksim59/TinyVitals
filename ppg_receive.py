#!/usr/bin/env python3
"""
TinyVitals — receive PPG lines from ESP32 (or Nano) over USB serial.

Device sketch prints:
  BPM,<n>
  ALERT,BPM,<n>
  STATUS,READY | STATUS,NO_SIGNAL

Usage:
  python3 ppg_receive.py
  python3 ppg_receive.py --port /dev/ttyUSB0

Install:  pip3 install pyserial
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Install pyserial:  pip3 install pyserial", file=sys.stderr)
    sys.exit(1)


def find_serial_port() -> str | None:
    """Prefer ESP32 / Arduino USB serial adapters."""
    ports = list(list_ports.comports())
    for p in ports:
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if any(
            k in desc
            for k in (
                "esp32",
                "cp210",
                "ch340",
                "ch341",
                "ft232",
                "arduino",
                "usb serial",
                "silicon labs",
            )
        ):
            return p.device
    for p in ports:
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            return p.device
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description="Read PPG BPM from ESP32 / Arduino")
    ap.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=9600)
    args = ap.parse_args()

    port = args.port or find_serial_port()
    if not port:
        print("No serial device found. Plug in the ESP32, or pass --port /dev/ttyUSB0")
        sys.exit(1)

    print(f"Opening {port} at {args.baud} baud...")
    ser = serial.Serial(port, args.baud, timeout=1)
    time.sleep(2.0)  # board may reset on open; wait for STATUS,READY
    ser.reset_input_buffer()

    print("Listening for BPM from ESP32 (Ctrl+C to stop)\n")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            tag = parts[0].upper()

            if tag == "BPM" and len(parts) >= 2:
                bpm = int(parts[1])
                print(f"Heart rate: {bpm} BPM")
                # TODO: log, push notification, combine with imu_test breathing alerts

            elif tag == "ALERT" and len(parts) >= 3:
                bpm = int(parts[2])
                print(f"*** ALERT: BPM {bpm} out of range — check now ***")

            elif tag == "STATUS":
                print(f"ESP32: {parts[1] if len(parts) > 1 else line}")

            else:
                print(f"(raw) {line}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
