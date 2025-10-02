// ESP32-WROOM-32D version: dual hardware I2C for AS5600 encoders
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

// ================= ESP32 I2C =================
#define SDA_HW0 21 // I2C0 SDA
#define SCL_HW0 22 // I2C0 SCL
#define SDA_HW1 25 // I2C1 SDA (choose available pins)
#define SCL_HW1 26 // I2C1 SCL

TwoWire I2C0 = TwoWire(0);
TwoWire I2C1 = TwoWire(1);

// ================= AS5600 =================
const byte AS5600_ADDR = 0x36; // Both encoders use same address
const uint8_t ANGLE_MSB = 0x0E;
const uint8_t ANGLE_LSB = 0x0F;

// ================= Robot parameters =================
const float track_radius = 0.055;
const float track_base   = 0.246;
const int steps_per_rev  = 200;
const int microstepping  = 1;
const float gear_ratio   = 1.0;

// ================= Pin driver =================
#define EN_R 12
#define DIR_R 13
#define STEP_R 14
#define EN_L 15
#define DIR_L 16
#define STEP_L 17

// ================= Stepper =================
AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

// ================= Stato robot =================
float v_lin = 0.0;
float v_ang = 0.0;
float x = 0.0, y = 0.0, th = 0.0;
float last_angle_left  = 0.0;
float last_angle_right = 0.0;

// ================= Funzioni =================
template <typename T>
float readAngleI2C(T &bus, byte addr) {
  bus.beginTransmission(addr);
  bus.write(ANGLE_MSB);
  if (bus.endTransmission(false) != 0) return NAN;
  if (bus.requestFrom(addr, (uint8_t)2) == 2) {
    uint16_t raw = (bus.read() << 8) | bus.read();
    raw &= 0x0FFF;
    return raw * 360.0 / 4096.0;
  }
  return NAN;
}

float deltaAngle(float last, float current) {
  float d = current - last;
  if (d > 180.0)  d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

long velToSteps(float v) {
  float track_circ = 2 * PI * track_radius;
  float rev_per_s = v / track_circ;
  return (long)(rev_per_s * steps_per_rev * microstepping * gear_ratio);
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  I2C0.begin(SDA_HW0, SCL_HW0, 400000L);
  I2C1.begin(SDA_HW1, SCL_HW1, 400000L);

  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);

  stepperL.setMaxSpeed(500);
  stepperR.setMaxSpeed(500);
  stepperL.setPinsInverted(true, false, false);

  last_angle_left  = readAngleI2C(I2C0, AS5600_ADDR);
  last_angle_right = readAngleI2C(I2C1, AS5600_ADDR);

  Serial.println("Robot pronto a ricevere comandi e trasmettere odometria...");
}

// ================= Loop =================
void loop() {
  // --- Gestione comandi da seriale ---
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("V")) {
      line.remove(0, 1);
      line.trim();
      int spaceIndex = line.indexOf(' ');
      if (spaceIndex > 0) {
        v_lin = line.substring(0, spaceIndex).toFloat();
        v_ang = line.substring(spaceIndex + 1).toFloat();
        float v_r = v_lin + (v_ang * track_base / 2.0);
        float v_l = v_lin - (v_ang * track_base / 2.0);
        long steps_r = velToSteps(v_r);
        long steps_l = velToSteps(v_l);
        stepperR.setSpeed(steps_r);
        stepperL.setSpeed(steps_l);
        Serial.print("CMD: V_lin=");
        Serial.print(v_lin, 3);
        Serial.print(" V_ang=");
        Serial.println(v_ang, 3);
      }
    }
  }
  // --- Movimento motori ---
  stepperL.runSpeed();
  stepperR.runSpeed();
  // --- Aggiorna odometria ogni 100ms ---
  static unsigned long lastOdo = 0;
  if (millis() - lastOdo >= 100) {
    lastOdo = millis();
    float angle_left  = readAngleI2C(I2C0, AS5600_ADDR);
    float angle_right = readAngleI2C(I2C1, AS5600_ADDR);
    if (!isnan(angle_left) && !isnan(angle_right)) {
      float dtheta_left  = deltaAngle(last_angle_left, angle_left);
      float dtheta_right = deltaAngle(last_angle_right, angle_right);
      last_angle_left  = angle_left;
      last_angle_right = angle_right;
      float dL = (dtheta_left  / 360.0) * (2.0 * PI * track_radius);
      float dR = (dtheta_right / 360.0) * (2.0 * PI * track_radius);
      static float pos_left_rad  = 0.0;
      static float pos_right_rad = 0.0;
      pos_left_rad  += dtheta_left  * PI / 360.0;
      pos_right_rad += dtheta_right * PI / 360.0;
      float dS = (dR + dL) / 2.0;
      float dTh = (dR - dL) / track_base;
      x  += dS * cos(th + dTh / 2.0);
      y  += dS * sin(th + dTh / 2.0);
      th += dTh;
      if (th > PI) th -= 2.0 * PI;
      if (th < -PI) th += 2.0 * PI;
      // --- Stampa odometria ---
      Serial.print("O ");
      Serial.print(x, 3);
      Serial.print(" ");
      Serial.print(y, 3);
      Serial.print(" ");
      Serial.println(th, 3);
      // --- Stampa posizione joint ---
      Serial.print("J ");
      Serial.print(pos_left_rad, 6);
      Serial.print(" ");
      Serial.println(pos_right_rad, 6);
    }
  }
}
