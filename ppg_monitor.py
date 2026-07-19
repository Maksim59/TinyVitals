#!/usr/bin/env python3
"""
TinyVitals — simple PPG (pulse) monitor for Raspberry Pi.

Pi has NO analog pins. This uses an ADS1115 I2C ADC to read the photodiode.

Wiring (Raspberry Pi 5, 3.3 V only — do not use 5 V on GPIO/ADC):

  Red LED (transmissive illuminator)
    Pi GPIO17 ── 220 Ω ── LED anode
    LED cathode ────────── GND

  Photodiode (opposite side of finger from the LED)
    3.3 V ── photodiode cathode
    photodiode anode ──┬── ADS1115 A0
                       └── 10 kΩ ── GND
    (If the signal is inverted / tiny, swap the 10k to pull-up to 3.3V
     and tie anode toward GND through the diode — see notes below.)

  ADS1115
    VDD → 3.3 V
    GND → GND
    SDA → Pi GPIO2 (SDA1)
    SCL → Pi GPIO3 (SCL1)
    ADDR → GND  (I2C address 0x48)

  Optional buzzer / alarm
    Pi GPIO27 → buzzer +
    buzzer − → GND
    (or use a transistor if the buzzer draws more than a few mA)

Enable I2C: sudo raspi-config → Interface Options → I2C → Yes
Install deps:  sudo apt install python3-pip
               pip3 install adafruit-blinka adafruit-circuitpython-ads1x15

Run:  python3 ppg_monitor.py
"""

from __future__ import annotations

import time
import board
import busio
import digitalio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# --- pins / thresholds (mirrors your Arduino sketch) ---
LED_PIN = board.D17
ALARM_PIN = board.D27

ALPHA = 0.05          # baseline EMA (same idea as your sketch)
PEAK_THRESH = 80.0    # filtered units above baseline = "beat" (tune this!)
MIN_INTERVAL_S = 0.30 # ignore intervals < 300 ms (>200 BPM impossible)
BPM_LOW = 70
BPM_HIGH = 170
STALE_S = 3.0         # no beat for this long → freeze/clear BPM
SAMPLE_DT = 0.01      # ~100 Hz loop


def sound_alarm(pin: digitalio.DigitalInOut, n: int = 2) -> None:
    for _ in range(n):
        pin.value = True
        time.sleep(0.4)
        pin.value = False
        time.sleep(1.0)


def main() -> None:
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    ads.gain = 1  # +/-4.096 V — good for 0–3.3 V photodiode swing
    light = AnalogIn(ads, ADS.P0)

    led = digitalio.DigitalInOut(LED_PIN)
    led.direction = digitalio.Direction.OUTPUT
    led.value = True  # keep LED on for continuous PPG

    alarm = digitalio.DigitalInOut(ALARM_PIN)
    alarm.direction = digitalio.Direction.OUTPUT
    alarm.value = False

    # Warm up baseline (your setup() analogRead)
    baseline = 0.0
    for _ in range(50):
        baseline = 0.9 * baseline + 0.1 * light.value
        time.sleep(0.01)

    last_beat_t = time.monotonic()
    was_above = False
    bpm = 0
    alerted = False

    print("PPG monitor running. Place fingertip between LED and photodiode.")
    print("Raw ADC is 0–32767. Tune PEAK_THRESH if beats are missed/noisy.\n")

    while True:
        raw = float(light.value)  # 16-bit ADS reading (0..32767)
        baseline = (ALPHA * raw) + ((1.0 - ALPHA) * baseline)
        filtered = raw - baseline

        is_above = filtered > PEAK_THRESH
        now = time.monotonic()

        # Rising edge across threshold = candidate beat
        if is_above and not was_above:
            interval = now - last_beat_t
            if interval > MIN_INTERVAL_S:
                last_beat_t = now
                bpm = int(60.0 / interval)
                print(f"BPM={bpm:3d}  filtered={filtered:7.1f}  raw={raw:.0f}")
                alerted = False  # allow a new alarm if BPM goes bad again

        # Stale signal (finger removed / lost lock)
        if now - last_beat_t > STALE_S:
            bpm = 0
            alerted = False

        if bpm != 0 and (bpm < BPM_LOW or bpm > BPM_HIGH) and not alerted:
            print(f"*** ALERT: BPM={bpm} out of range ({BPM_LOW}-{BPM_HIGH}) ***")
            sound_alarm(alarm, 2)
            alerted = True

        was_above = is_above
        time.sleep(SAMPLE_DT)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
