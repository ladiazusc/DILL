/*
  Electromagnetic CubeSat Soft Docking — Sensor Suite & Logger
  -----------------------------------------------------------
  Flow:
    1) Reads 2x ultrasonics (agree < 4 cm for a few samples).
    2) When agreed, closes "motor power" relay (feeds L298N VS) and energizes magnets.
    3) Continuously monitors +Y (impact axis) force from MPU6050; if > 5 N → FAIL:
         - Opens motor power relay (cuts L298N VS), magnets off, pogo OFF, OLED says FAIL.
    4) If no fail after 5 s -> closes "pogo" relay to pass 12 V to pogo pins (SUCCESS).
  Logging:
    - Streams true CSV over Serial (if enabled) with header: time, distances, ax, Fx, states, etc.
    - Title can be typed at startup (5 s window); included in the CSV header and OLED first line.

  IMPORTANT ABOUT PINS D0/D1:
    - D0/D1 are the hardware UART used by the USB Serial.
    - If you need ENA2/ENB2 on D0/D1, you *cannot* also use USB Serial CSV.
    - Use the compile-time switch below to choose:
        USE_HWSERIAL_CSV = 1  -> keep Serial CSV; ENA2/ENB2 are remapped to safe pins.
        USE_HWSERIAL_CSV = 0  -> disable Serial CSV; you may keep ENA2=0 and ENB2=1.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>

// ========================== BUILD-TIME SWITCHES ==========================
#define USE_HWSERIAL_CSV 1   // 1 = print CSV over USB Serial; 0 = no Serial (safe to use D0/D1 for L298N#2)

// ========================== DISPLAY (OLED) ===============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ========================== SENSORS (I2C + Ultrasonics) ==================
MPU6050 mpu;
const float g_const = 9.80665f;   // [m/s^2]
const float mass_kg = 0.10f;      // [kg] update with your probe mass
float ax_bias = 0.0f;             // [m/s^2] resting ax calibration

// Ultrasonic pins (per your spec)
const int trigPin1 = 11;
const int echoPin1 = 12;
const int trigPin2 = 10;
const int echoPin2 = 9;

// ========================== RELAYS ======================================
// IN1: motor-driver power cut → feeds L298N VS through NO contact
const int RELAY_MOTOR_PIN = 13;
// IN2: pogo-pin 12V pass
const int RELAY_POGO_PIN  = 2;

// Adjust these if your relay board is active-LOW
const bool RELAY_ACTIVE   = HIGH;
const bool RELAY_INACTIVE = LOW;

// ========================== L298N #1 (Magnet A & B) ======================
const int ENA1 = 8;
const int IN1_1 = 7;
const int IN2_1 = 6;
const int IN3_1 = 5;
const int IN4_1 = 4;
const int ENB1 = 3;

// ========================== L298N #2 (Optional) ==========================
// If you *must* use D0/D1 (0/1), set USE_HWSERIAL_CSV to 0.
// Otherwise we remap ENA2/ENB2 to safe pins so Serial works.
#if USE_HWSERIAL_CSV
  // Safe mapping (keeps USB Serial functional)
  const int ENA2 = 9;     // PWM-capable
  const int IN1_2 = A0;   // 14
  const int IN2_2 = A1;   // 15
  const int IN3_2 = A2;   // 16
  const int IN4_2 = A3;   // 17
  const int ENB2 = 10;    // PWM-capable
#else
  // Your requested mapping using D0/D1 (USB Serial disabled)
  const int ENA2 = 0;     // D0 (RX) — used as digital pin
  const int IN1_2 = A0;   // 14
  const int IN2_2 = A1;   // 15
  const int IN3_2 = A2;   // 16
  const int IN4_2 = A3;   // 17
  const int ENB2 = 1;     // D1 (TX) — used as digital pin
#endif

// ========================== BEHAVIOR CONFIG ==============================
const float DIST_THRESH_CM   = 4.0f;     // agreement threshold [cm]
const uint8_t AGREE_COUNT    = 3;        // require N consecutive agrees
const unsigned long MAG_HOLD_MS = 5000;  // hold magnets before pogo [ms]
const float FORCE_LIMIT_N    = 5.0f;     // safety limit on +Y [N]
const unsigned long LOG_PERIOD_MS = 50;  // CSV cadence [ms]

// ========================== STATE MACHINE ================================
enum RunState { IDLE, MAG_ON, POGO_ON, DONE, FAIL };
RunState state = IDLE;
unsigned long stateEnterMs = 0;

// Run metadata
String runTitle = "untitled";
uint8_t agreeCounter = 0;
unsigned long lastLogMs = 0;

// utilities
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout (~5 m)
  if (us <= 0) return NAN;
  return us * 0.0343f * 0.5f;
}

void relayWrite(int pin, bool on) {
  digitalWrite(pin, on ? RELAY_ACTIVE : RELAY_INACTIVE);
}

// Magnet controls via L298N #1 
void magnetA_off() { digitalWrite(IN1_1, LOW); digitalWrite(IN2_1, LOW); digitalWrite(ENA1, LOW); }
void magnetA_forward() { digitalWrite(IN1_1, HIGH); digitalWrite(IN2_1, LOW); digitalWrite(ENA1, HIGH); }
void magnetA_reverse() { digitalWrite(IN1_1, LOW);  digitalWrite(IN2_1, HIGH); digitalWrite(ENA1, HIGH); }

void magnetB_off() { digitalWrite(IN3_1, LOW); digitalWrite(IN4_1, LOW); digitalWrite(ENB1, LOW); }
void magnetB_forward() { digitalWrite(IN3_1, HIGH); digitalWrite(IN4_1, LOW); digitalWrite(ENB1, HIGH); }
void magnetB_reverse() { digitalWrite(IN3_1, LOW);  digitalWrite(IN4_1, HIGH); digitalWrite(ENB1, HIGH); }

void magnets_all_off() { magnetA_off(); magnetB_off(); }

// Initialize L298N #2
void driver2_init() {
  pinMode(ENA2, OUTPUT); pinMode(IN1_2, OUTPUT); pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT); pinMode(IN4_2, OUTPUT); pinMode(ENB2, OUTPUT);
  digitalWrite(ENA2, LOW); digitalWrite(ENB2, LOW);
  digitalWrite(IN1_2, LOW); digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW); digitalWrite(IN4_2, LOW);
}

void oledStatus(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println(runTitle);
  display.setCursor(0, 16); display.println(line1);
  display.setCursor(0, 26); display.println(line2);
  display.display();
}

// Setup
void setup() {
#if USE_HWSERIAL_CSV
  Serial.begin(115200);
#endif
  Wire.begin();

  // OLED
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay(); display.display();
  }

  // MPU6050
  mpu.initialize();
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // default ±2g

  // Ultrasonics
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);

  // Relays
  pinMode(RELAY_MOTOR_PIN, OUTPUT);
  pinMode(RELAY_POGO_PIN, OUTPUT);
  relayWrite(RELAY_MOTOR_PIN, false); // motor power OFF
  relayWrite(RELAY_POGO_PIN, false);  // pogo OFF

  // L298N #1
  pinMode(ENA1, OUTPUT); pinMode(IN1_1, OUTPUT); pinMode(IN2_1, OUTPUT);
  pinMode(IN3_1, OUTPUT); pinMode(IN4_1, OUTPUT); pinMode(ENB1, OUTPUT);
  magnets_all_off();

  // L298N #2
  driver2_init();

  // Title prompt (5 s)
  oledStatus("Awaiting title...", "Type & press Enter");
#if USE_HWSERIAL_CSV
  Serial.println(F("# Enter run title (5 s), then press Enter:"));
  unsigned long t0 = millis();
  while (millis() - t0 < 5000UL) {
    if (Serial.available()) {
      runTitle = Serial.readStringUntil('\n'); runTitle.trim();
      if (runTitle.length() == 0) runTitle = "untitled";
      break;
    }
  }
#endif

  // Calibrate ax bias (assumes stationary, correct orientation)
  const int N = 200; long sumAx_times1e4 = 0;
  for (int i = 0; i < N; ++i) {
    int16_t axr, ayr, azr; mpu.getAcceleration(&axr, &ayr, &azr);
    float ax_mps2 = (float)axr / 16384.0f * g_const;
    sumAx_times1e4 += (long)(ax_mps2 * 10000.0f);
    delay(5);
  }
  ax_bias = ((float)sumAx_times1e4 / 10000.0f) / N;

  // CSV header
#if USE_HWSERIAL_CSV
  Serial.println(F("# CSV, title="));
  Serial.println(runTitle);
  Serial.println(F("time_ms,dist1_cm,dist2_cm,agree_flag,ax_mps2,force_x_N,state,relay_motor,relay_pogo,magA,magB"));
#endif

  state = IDLE; stateEnterMs = millis();
  oledStatus("IDLE", "Waiting <4 cm agree");
}

// Loop
void loop() {
  const unsigned long now = millis();

  // Read Sensors
  const float d1 = readUltrasonicCM(trigPin1, echoPin1);
  const float d2 = readUltrasonicCM(trigPin2, echoPin2);

  int16_t axr, ayr, azr; mpu.getAcceleration(&axr, &ayr, &azr);
  const float ax = (float)axr / 16384.0f * g_const;         // +Y axis (impact)
  const float Fx = mass_kg * fabs(ax - ax_bias);            // [N] |Δax| * m

  // Distance Agreeance
  const bool valid_d1 = (!isnan(d1) && d1 > 0);
  const bool valid_d2 = (!isnan(d2) && d2 > 0);
  const bool agreeNow = valid_d1 && valid_d2 &&
                        (d1 < DIST_THRESH_CM) && (d2 < DIST_THRESH_CM);
  if (agreeNow) { if (agreeCounter < 255) agreeCounter++; }
  else          { if (agreeCounter > 0)   agreeCounter--; }
  const bool agreeStable = (agreeCounter >= AGREE_COUNT);

  // Force Limit
  if (Fx > FORCE_LIMIT_N && state != FAIL && state != DONE) {
    relayWrite(RELAY_MOTOR_PIN, false); // kill L298N VS
    magnets_all_off();
    relayWrite(RELAY_POGO_PIN, false);
    state = FAIL; stateEnterMs = now;
    oledStatus("FAIL: Force > limit", "Motor power OFF");
  }

  // --- State machine ---
  switch (state) {
    case IDLE:
      relayWrite(RELAY_MOTOR_PIN, false);
      relayWrite(RELAY_POGO_PIN, false);
      magnets_all_off();
      if (agreeStable) {
        relayWrite(RELAY_MOTOR_PIN, true);   // feed VS to L298N
        magnetA_forward(); magnetB_forward();
        state = MAG_ON; stateEnterMs = now;
        oledStatus("MAG_ON", "Holding 5s before pogo");
      }
      break;

    case MAG_ON:
      relayWrite(RELAY_MOTOR_PIN, true);
      if (now - stateEnterMs >= MAG_HOLD_MS) {
        relayWrite(RELAY_POGO_PIN, true);   // pass 12 V to pogo pins
        state = POGO_ON; stateEnterMs = now;
        oledStatus("POGO_ON", "Power pass → SUCCESS");
      }
      break;

    case POGO_ON:
      if (now - stateEnterMs >= 2000UL) {
        state = DONE; stateEnterMs = now;
        oledStatus("DONE", "Sequence complete");
      }
      break;

    case DONE:
      // Keep as-is; optionally relax magnets or open relays here if desired
      break;

    case FAIL:
      // Remain safe until reset
      break;
  }

  // Serial - CSV logging? Might switch over to MAX6675 method we did in Mechops
#if USE_HWSERIAL_CSV
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    // time_ms,dist1_cm,dist2_cm,agree_flag,ax_mps2,force_x_N,state,relay_motor,relay_pogo,magA,magB
    Serial.print(now); Serial.print(',');
    Serial.print(valid_d1 ? d1 : -1.0f); Serial.print(',');
    Serial.print(valid_d2 ? d2 : -1.0f); Serial.print(',');
    Serial.print(agreeStable ? 1 : 0); Serial.print(',');
    Serial.print(ax, 4); Serial.print(',');
    Serial.print(Fx, 3); Serial.print(',');

    switch (state) {
      case IDLE:   Serial.print("IDLE"); break;
      case MAG_ON: Serial.print("MAG_ON"); break;
      case POGO_ON:Serial.print("POGO_ON"); break;
      case DONE:   Serial.print("DONE"); break;
      case FAIL:   Serial.print("FAIL"); break;
    }
    Serial.print(',');

    const int rm = (digitalRead(RELAY_MOTOR_PIN) == RELAY_ACTIVE) ? 1 : 0;
    const int rp = (digitalRead(RELAY_POGO_PIN)  == RELAY_ACTIVE) ? 1 : 0;
    const int mA = digitalRead(ENA1) ? 1 : 0;
    const int mB = digitalRead(ENB1) ? 1 : 0;

    Serial.print(rm); Serial.print(',');
    Serial.print(rp); Serial.print(',');
    Serial.print(mA); Serial.print(',');
    Serial.println(mB);
  }
#endif
}
