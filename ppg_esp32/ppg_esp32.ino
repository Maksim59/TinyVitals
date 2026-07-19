/*
 * TinyVitals — simulated BPM for ESP32 (standalone demo only).
 * Not connected to the Raspberry Pi.
 *
 * Cover the light sensor on D15 → lock one BPM in 60..100,
 * then print values in [base, base+2] on USB Serial.
 *
 * Wiring:
 *   LED:  3V3 → round (anode), flat (cathode) → 220Ω → D23  (active LOW)
 *   Light sensor signal → D15
 */

const int LED_PIN = 23;
const int LIGHT_SENSOR = 15;
const int ALARM_PIN = 2;

const int COVER_ON  = 1200;
const int COVER_OFF = 2500;
const unsigned long LOCK_MS   = 400;
const unsigned long UNLOCK_MS = 1500;
const unsigned long BPM_PRINT_MS = 2000;

float smooth = 0;
bool locked = false;
int baseBpm = 0;
int lastShown = 0;
unsigned long coverStart = 0;
unsigned long uncoverStart = 0;
unsigned long lastPrint = 0;
bool timingCover = false;
bool timingUncover = false;

void ledOn()  { digitalWrite(LED_PIN, LOW); }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);
  ledOn();

  analogReadResolution(12);
  analogSetPinAttenuation(LIGHT_SENSOR, ADC_11db);

  Serial.begin(9600);
  randomSeed(esp_random());
  delay(500);

  long sum = 0;
  for (int i = 0; i < 30; i++) {
    sum += analogRead(LIGHT_SENSOR);
    delay(5);
  }
  smooth = sum / 30.0f;

  Serial.println("STATUS,READY");
}

void loop() {
  int raw = analogRead(LIGHT_SENSOR);
  smooth = 0.85f * smooth + 0.15f * raw;
  int level = (int)smooth;
  unsigned long now = millis();

  if (!locked) {
    if (level < COVER_ON) {
      if (!timingCover) {
        timingCover = true;
        coverStart = now;
      } else if (now - coverStart >= LOCK_MS) {
        locked = true;
        timingCover = false;
        timingUncover = false;
        baseBpm = random(60, 101);
        lastShown = baseBpm;
        lastPrint = now;
        Serial.print("STATUS,LOCKED,");
        Serial.println(baseBpm);
        Serial.print("BPM,");
        Serial.println(lastShown);
      }
    } else {
      timingCover = false;
    }
  } else {
    if (level > COVER_OFF) {
      if (!timingUncover) {
        timingUncover = true;
        uncoverStart = now;
      } else if (now - uncoverStart >= UNLOCK_MS) {
        locked = false;
        baseBpm = 0;
        timingUncover = false;
        timingCover = false;
        Serial.println("STATUS,NO_SIGNAL");
      }
    } else {
      timingUncover = false;
    }

    if (locked && baseBpm > 0 && (now - lastPrint >= BPM_PRINT_MS)) {
      lastPrint = now;
      if (random(0, 100) < 30)
        lastShown = baseBpm + random(0, 3);
      Serial.print("BPM,");
      Serial.println(lastShown);
    }
  }

  delay(20);
}
