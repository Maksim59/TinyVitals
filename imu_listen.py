#!/usr/bin/env python3
"""Receive IMU UDP from the QNX Pi and print like imu_test.c.

  python3 imu_listen.py

On the Pi:
  ./imu_test <THIS_LAPTOP_IP>
"""

import socket

POS_NAMES = {
    0: "BACK (safe)",
    1: "STOMACH - WARNING",
    2: "LEFT SIDE - caution",
    3: "RIGHT SIDE - caution",
    4: "unknown/transitional",
}

PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", PORT))
print(f"Listening on UDP :{PORT}  (Ctrl+C to stop)\n")

while True:
    data, _addr = sock.recvfrom(256)
    raw = data.decode("utf-8", errors="ignore").strip()
    if not raw.startswith("IMU,"):
        print(raw)
        continue
    try:
        parts = dict(p.split("=", 1) for p in raw.split(",")[1:] if "=" in p)
        breath = float(parts.get("breath", 0))
        pos = int(parts.get("pos", 4))
        apnea = int(parts.get("apnea", 0))
        name = POS_NAMES.get(pos, "unknown/transitional")
        line = f"{name} | breath {breath:.0f}/min"
        if apnea:
            line += " | APNEA"
        print(line, flush=True)
    except (ValueError, TypeError):
        print(raw)
