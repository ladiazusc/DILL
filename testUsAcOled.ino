#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// US Distance Sensor
const int trigPin1 = 11; 
const int echoPin1 = 12;

const int trigPin2 = 10; 
const int echoPin2 = 9;

// Accelerometer
MPU6050 mpu;
const float mass = 0.1;         // Mass [kg]
const float g = 9.80665;

// Relay Setup -- Will use 2x L298N Motor Driver Modules for magnets. Will use one relay to pass power to motor driver board @ 12V via Power supply
const int relay1Pin = 8;        // Magnet 1
const int relay2Pin = 9;        // Magnet 2
unsigned long lastToggleTime1 = 0;
unsigned long lastToggleTime2 = 0;
bool magnet1State = false;
bool magnet2State = false;
const unsigned long onTime  = 2000;
const unsigned long offTime = 2000;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // MPU6050 Init
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Ultrasonic Init
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Relays Init
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);

  Serial.println("time_ms distance_cm force_N M1_state M2_state");
}

void loop() {
  unsigned long t = millis();

  // ---- Ultrasonic ----
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance_cm = duration * 0.0343 / 2.0;

  // ---- MPU6050 ----
  int16_t ax_raw, ay_raw, az_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  float accelScale = 16384.0;
  float ax = (float)ax_raw / accelScale * g;
  float ay = (float)ay_raw / accelScale * g;
  float az = (float)az_raw / accelScale * g;
  float netAccel = sqrt(ax * ax + ay * ay + az * az);
  float force = mass * netAccel;

  // ---- Magnet 1 Toggle ----
  if (magnet1State && (t - lastToggleTime1 >= onTime)) {
    magnet1State = false;
    digitalWrite(relay1Pin, LOW);
    lastToggleTime1 = t;
  } else if (!magnet1State && (t - lastToggleTime1 >= offTime)) {
    magnet1State = true;
    digitalWrite(relay1Pin, HIGH);
    lastToggleTime1 = t;
  }

  // ---- Magnet 2 Toggle (offset for demonstration) ----
  if (magnet2State && (t - lastToggleTime2 >= onTime)) {
    magnet2State = false;
    digitalWrite(relay2Pin, LOW);
    lastToggleTime2 = t;
  } else if (!magnet2State && (t - lastToggleTime2 >= offTime + 1000)) {
    magnet2State = true;
    digitalWrite(relay2Pin, HIGH);
    lastToggleTime2 = t;
  }

  // ---- Serial Output ----
  Serial.print(t); Serial.print(" ");
  Serial.print(distance_cm, 1); Serial.print(" ");
  Serial.print(force, 2); Serial.print(" ");
  Serial.print(magnet1State ? "ON " : "OFF ");
  Serial.println(magnet2State ? "ON" : "OFF");

  // ---- OLED ----
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("G18 Dual Magnet Summary");

  display.setCursor(0, 16);
  display.print("Distance: "); display.print(distance_cm, 1); display.println(" cm");
  display.setCursor(0, 26);
  display.print("Force: "); display.print(force, 2); display.println(" N");
  display.setCursor(0, 36);
  display.print("M1: "); display.println(magnet1State ? "ON" : "OFF");
  display.setCursor(0, 46);
  display.print("M2: "); display.println(magnet2State ? "ON" : "OFF");
  display.display();

  delay(100);
}
