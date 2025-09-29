#include <AccelStepper.h>
#include <Wire.h>

// ================= Parametri robot =================
const float track_radius = 0.055;   // raggio equivalente del cingolo [m]
const float track_base   = 0.246;   // distanza tra i due cingoli [m]
const int steps_per_rev  = 200;     // passi motore per giro (1.8°)
const int microstepping  = 1;       // microstepping driver
const float gear_ratio   = 1.0;     // eventuale riduzione

// ================= Pin driver =================
// RIGHT track
#define EN_R 2 //12
#define DIR_R 5 //11
#define STEP_R 6 //10
// LEFT track
#define EN_L 4 //4
#define DIR_L 3 //3
#define STEP_L 9 //9

// ================= Stepper =================
AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

// ================= AS5600 =================
const byte AS5600_LEFT  = 0x36;  
const byte AS5600_RIGHT = 0x37;   // il secondo con ADDR alto

// ================= Stato robot =================
float v_lin = 0.0;
float v_ang = 0.0;

float x = 0.0, y = 0.0, th = 0.0; 
float last_angle_left  = 0.0;
float last_angle_right = 0.0;

// ================= Funzione conversione velocità -> step/s =================
long velToSteps(float v) {
  float track_circ = 2 * PI * track_radius;
  float rev_per_s = v / track_circ;  
  float steps_per_s = rev_per_s * steps_per_rev * microstepping * gear_ratio;
  return (long)steps_per_s;
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  Serial.println("Robot pronto a ricevere comandi e trasmettere odometria...");

  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  digitalWrite(EN_L, LOW); // enable attivo basso
  digitalWrite(EN_R, LOW);

  stepperL.setMaxSpeed(500);
  stepperR.setMaxSpeed(500);

  stepperL.setPinsInverted(true, false, false);

  Wire.begin();
  Wire.setClock(400000L);

  last_angle_left  = readAngle(AS5600_LEFT);
  last_angle_right = readAngle(AS5600_RIGHT);
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

  // --- Aggiorna odometria ---
  static unsigned long lastOdo = 0;
  if (millis() - lastOdo >= 100) {  // aggiorna ogni 100 ms
    lastOdo = millis();

    float angle_left  = readAngle(AS5600_LEFT);
    float angle_right = readAngle(AS5600_RIGHT);

    float dtheta_left  = deltaAngle(last_angle_left, angle_left);
    float dtheta_right = deltaAngle(last_angle_right, angle_right);

    last_angle_left  = angle_left;
    last_angle_right = angle_right;

        // Spostamenti lineari delle ruote
    float dL = (dtheta_left  / 360.0) * (2.0 * PI * track_radius);
    float dR = (dtheta_right / 360.0) * (2.0 * PI * track_radius);

    // --- Posizioni cumulative delle ruote in radianti ---
    static float pos_left_rad  = 0.0;
    static float pos_right_rad = 0.0;
    pos_left_rad  += dtheta_left  * PI / 180.0;  // deg → rad
    pos_right_rad += dtheta_right * PI / 180.0;  // deg → rad


    float dS = (dR + dL) / 2.0;
    float dTh = (dR - dL) / track_base;

    x  += dS * cos(th + dTh / 2.0);
    y  += dS * sin(th + dTh / 2.0);
    th += dTh;

    if (th > PI) th -= 2.0 * PI;
    if (th < -PI) th += 2.0 * PI;

    Serial.print("O ");
    Serial.print(x, 3);
    Serial.print(" ");
    Serial.print(y, 3);
    Serial.print(" ");
    Serial.println(th, 3);
    Serial.print("J ");
    Serial.print(pos_left_rad, 6);
    Serial.print(" ");
    Serial.println(pos_right_rad, 6);
  }
}

// ================= Funzioni AS5600 =================
float readAngle(byte addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x0E); // ANGLE (MSB)
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)2);

  if (Wire.available() == 2) {
    uint16_t raw = (Wire.read() << 8) | Wire.read();
    raw = raw & 0x0FFF; 
    return raw * 360.0 / 4096.0;
  }
  return 0.0;
}

float deltaAngle(float last, float current) {
  float d = current - last;
  if (d > 180.0)  d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}
