/*
 * TinyVitals — PPG pulse monitor for Arduino Nano
 * Sends BPM (and optional debug) over USB serial to a Raspberry Pi.
 *
 * Wiring (Arduino Nano, 5 V USB-powered):
 *
 *   Red LED (one side of finger)
 *     D9 ── 220 Ω ── LED anode
 *     LED cathode ── GND
 *
 *   Photodiode (other side of finger — light through the tip)
 *     5V ── photodiode cathode
 *     photodiode anode ──┬── A3
 *                        └── 10 kΩ ── GND
 *
 *   Optional buzzer
 *     D2 ── buzzer +
 *     buzzer − ── GND
 *
 * Pi side: plug Nano in via USB, run:  python3 ppg_receive.py
 *
 * Serial protocol (9600 baud), one line per event:
 *   BPM,<value>          heart rate update
 *   ALERT,BPM,<value>    out-of-range BPM
 *   STATUS,NO_SIGNAL     no beat for a few seconds
 */

const int LED_PIN = 9;          // PWM-capable; keep LED on for PPG
const int LIGHT_SENSOR = A3;
const int ALARM_PIN = 2;

const float ALPHA = 0.05;       // baseline EMA
const float PEAK_THRESH = 80.0; // tune: raise if noisy, lower if missing beats
const unsigned long MIN_INTERVAL_MS = 300;  // ignore >200 BPM spikes
const int BPM_LOW = 70;
const int BPM_HIGH = 170;
const unsigned long STALE_MS = 3000;

float baseline = 0;
unsigned long lastBeatTime = 0;
bool wasAbove = false;
int bpm = 0;
bool alerted = false;

void soundAlarm(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(ALARM_PIN, HIGH);
    delay(400);
    digitalWrite(ALARM_PIN, LOW);
    delay(1000);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

  // Full brightness (Nano D9 supports PWM)
  analogWrite(LED_PIN, 255);

  Serial.begin(9600);
  delay(500);

  // Warm up baseline so the first samples aren't junk
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(LIGHT_SENSOR);
    delay(10);
  }
  baseline = sum / 50.0;
  lastBeatTime = millis();

  Serial.println("STATUS,READY");
}

void loop() {
  int raw = analogRead(LIGHT_SENSOR);
  baseline = (ALPHA * raw) + ((1.0 - ALPHA) * baseline);
  float filtered = raw - baseline;

  bool isAbove = (filtered > PEAK_THRESH);
  unsigned long now = millis();

  // Rising edge across threshold = new beat
  if (isAbove && !wasAbove) {
    unsigned long interval = now - lastBeatTime;

    if (interval > MIN_INTERVAL_MS) {
      lastBeatTime = now;
      bpm = (int)(60000UL / interval);
      alerted = false;

      Serial.print("BPM,");
      Serial.println(bpm);
    }
  }

  // Finger removed / lost optical lock
  if ((now - lastBeatTime) > STALE_MS) {
    if (bpm != 0) {
      bpm = 0;
      alerted = false;
      Serial.println("STATUS,NO_SIGNAL");
    }
  }

  if (bpm != 0 && (bpm < BPM_LOW || bpm > BPM_HIGH) && !alerted) {
    Serial.print("ALERT,BPM,");
    Serial.println(bpm);
    soundAlarm(2);
    alerted = true;
  }

  wasAbove = isAbove;
  delay(10);  // ~100 Hz sample loop
}
