// =====================================================================
//  MediFly — Firmware de vol pour drone quadricoptère
//  Groupe PLBD 19 — École Centrale Casablanca — Juin 2026
//
//  Carte cible : ESP32 DevKit V1 (Arduino Core 2.x — contraint par Bluepad32)
//
//  Capteurs & actionneurs :
//    - MPU6050   (IMU 6 axes)   I2C sur GPIO25 (SDA) / GPIO26 (SCL)
//    - BMP280    (baromètre)    même bus I2C
//    - GPS NEO-6M (positionnement)  UART2 — RX=GPIO16, TX=GPIO17 @ 9600 baud
//    - 4 ESC 30A (moteurs A2212 1400KV)  PWM LEDC @ 50 Hz, 16 bits
//        M1 (avant-gauche, CCW) -> GPIO18
//        M2 (avant-droit,  CW ) -> GPIO19
//        M3 (arrière-gauche, CW) -> GPIO21
//        M4 (arrière-droit, CCW) -> GPIO22
//    - Servo MG90 (treuil box)  PWM LEDC -> GPIO27
//    - Manette PS4 via Bluepad32 (Bluetooth Classic)
//
//  Sortie Serial @ 115200 baud — format parsé par la plateforme MediFly :
//    ARME | R:  0.2° P:  0.1° | ALT:0.14m T:24.4°C | GPS:FIX SAT:7 LAT:33.573100 LNG:-7.589800 HDG:74.3 SPD:0.0
// =====================================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Bluepad32.h>

// ---------------- BROCHES ----------------
static const int PIN_SDA      = 25;
static const int PIN_SCL      = 26;
static const int PIN_GPS_RX   = 16;   // RX ESP32 <- TX du NEO-6M
static const int PIN_GPS_TX   = 17;   // TX ESP32 -> RX du NEO-6M
static const int PIN_M1       = 18;
static const int PIN_M2       = 19;
static const int PIN_M3       = 21;
static const int PIN_M4       = 22;
static const int PIN_SERVO    = 27;

// ---------------- LEDC (PWM) ----------------
// Canal 0..3 pour les moteurs, canal 4 pour le servo. 50 Hz, résolution 16 bits.
static const int CH_M1=0, CH_M2=1, CH_M3=2, CH_M4=3, CH_SERVO=4;
static const int PWM_FREQ_HZ   = 50;
static const int PWM_RES_BITS  = 16;
// À 50 Hz, période = 20 ms = 65535 ticks (résolution 16 bits).
// Largeur d'impulsion en µs -> duty = (us * 65535) / 20000
static inline uint32_t usToDuty(uint32_t us){ return (us * 65535UL) / 20000UL; }

static const int ESC_US_MIN     = 1000;  // gaz mini
static const int ESC_US_INITIAL = 1050;  // armement
static const int ESC_US_MAX     = 2000;  // gaz max
static const int SERVO_US_UP    = 1100;  // treuil remonté (box accrochée)
static const int SERVO_US_DOWN  = 1900;  // treuil déployé (box au sol)

// ---------------- INSTANCES CAPTEURS ----------------
Adafruit_MPU6050 mpu;
Adafruit_BMP280  bmp;
TinyGPSPlus      gps;
HardwareSerial   GPSSerial(2);  // UART2

// ---------------- ÉTAT VOL ----------------
volatile bool armed         = false;   // armé / désarmé
volatile bool btConnected   = false;   // manette PS4 connectée ?
float baroSeaLevelhPa       = 1013.25; // pression de référence (sera calibrée au boot)
float altitudeOffset        = 0.0;     // altitude relative (point de décollage = 0)

// Consignes pilote (depuis manette PS4)
float setRoll = 0, setPitch = 0, setYaw = 0;
int   throttleUs = ESC_US_MIN;

// Mesures filtrées
float roll = 0, pitch = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
unsigned long lastImuMicros = 0;

// PID — coefficients (à régler aux étapes 3-4 de la validation)
struct Pid { float kp, ki, kd, integ, prevErr, outMin, outMax; };
Pid pidRoll  = { 1.20f, 0.020f, 0.50f, 0, 0, -250, 250 };
Pid pidPitch = { 1.20f, 0.020f, 0.50f, 0, 0, -250, 250 };
Pid pidYaw   = { 1.80f, 0.000f, 0.00f, 0, 0, -200, 200 };

// Failsafe
unsigned long lastBtSeenMs = 0;
static const unsigned long FAILSAFE_TIMEOUT_MS = 800;

// Cadence d'envoi télémétrie
unsigned long lastTelemMs = 0;
static const unsigned long TELEM_PERIOD_MS = 200; // 5 Hz

// =====================================================================
//   BLUEPAD32 — Callbacks manette PS4
// =====================================================================
ControllerPtr myCtrl = nullptr;

void onConnectedController(ControllerPtr ctl){
  myCtrl = ctl;
  btConnected = true;
  lastBtSeenMs = millis();
  Serial.println(F("[BT] Manette PS4 connectée"));
}

void onDisconnectedController(ControllerPtr ctl){
  if(ctl == myCtrl){ myCtrl = nullptr; btConnected = false; armed = false; }
  Serial.println(F("[BT] Manette PS4 déconnectée — DÉSARMEMENT failsafe"));
}

void processController(ControllerPtr ctl){
  if(!ctl || !ctl->isConnected()) return;
  lastBtSeenMs = millis();

  // Mapping Mode 2 :
  //  axisY() / axisX()  -> joystick gauche  : gaz (vertical), lacet (horizontal)
  //  axisRY()/ axisRX() -> joystick droit   : tangage (vertical), roulis (horizontal)
  // Bluepad32 renvoie un int dans [-512, 511] approximativement
  int gas   = -ctl->axisY();   // inversé : pousser le stick vers le haut -> gas+
  int yaw   =  ctl->axisX();
  int pitch =  -ctl->axisRY(); // inversé
  int roll_ =  ctl->axisRX();

  // Zone morte sur les axes
  auto deadband = [](int v, int dz){ return abs(v) < dz ? 0 : v; };
  gas   = deadband(gas,   30);
  yaw   = deadband(yaw,   30);
  pitch = deadband(pitch, 30);
  roll_ = deadband(roll_, 30);

  // Conversion en consignes
  // Gas : de [-512..511] -> [ESC_US_MIN..ESC_US_MAX] (centré ~ stick neutre = au repos)
  // Stick vers le bas = ESC_US_MIN ; vers le haut = ESC_US_MAX
  int gasNorm = constrain((gas + 512), 0, 1023);
  throttleUs  = ESC_US_MIN + (int)((long)gasNorm * (ESC_US_MAX - ESC_US_MIN) / 1023);

  setRoll  = (float)roll_ / 512.0f * 25.0f;   // ±25° max
  setPitch = (float)pitch / 512.0f * 25.0f;
  setYaw   = (float)yaw   / 512.0f * 180.0f;  // ±180°/s

  // Armement / désarmement
  static bool prevArmCombo = false;
  bool armCombo = ctl->l1() && ctl->r1();
  if(armCombo && !prevArmCombo){ armed = true; Serial.println(F("[CMD] ARME")); }
  prevArmCombo = armCombo;

  if(ctl->miscSelect() || ctl->miscBack()){
    armed = false;
    Serial.println(F("[CMD] DESARME (Options)"));
  }

  // Treuil box : croix haut/bas
  if(ctl->dpad() == DPAD_UP)   ledcWrite(CH_SERVO, usToDuty(SERVO_US_UP));
  if(ctl->dpad() == DPAD_DOWN) ledcWrite(CH_SERVO, usToDuty(SERVO_US_DOWN));
}

// =====================================================================
//   CAPTEURS
// =====================================================================
void calibrateGyro(){
  // 200 mesures stick au repos pour estimer le biais du gyroscope
  Serial.println(F("[CAL] Calibration gyroscope... (ne pas bouger le drone)"));
  long gx=0, gy=0, gz=0;
  const int N=200;
  for(int i=0; i<N; i++){
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    gx += g.gyro.x * 1000;
    gy += g.gyro.y * 1000;
    gz += g.gyro.z * 1000;
    delay(4);
  }
  gyroBiasX = (gx / (float)N) / 1000.0f;
  gyroBiasY = (gy / (float)N) / 1000.0f;
  gyroBiasZ = (gz / (float)N) / 1000.0f;
  Serial.print(F("[CAL] Biais gyro : "));
  Serial.print(gyroBiasX,4); Serial.print(F("  "));
  Serial.print(gyroBiasY,4); Serial.print(F("  "));
  Serial.println(gyroBiasZ,4);
}

void updateAttitude(){
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = micros();
  float dt = (lastImuMicros==0) ? 0.005f : (now - lastImuMicros) / 1.0e6f;
  if(dt > 0.1f) dt = 0.005f;
  lastImuMicros = now;

  // Angles depuis l'accéléromètre (référence statique)
  float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
  float rollAcc  = atan2f(ay, az) * 57.2958f;
  float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;

  // Intégration du gyroscope (référence dynamique) — biais retiré
  float gx = (g.gyro.x - gyroBiasX) * 57.2958f;
  float gy = (g.gyro.y - gyroBiasY) * 57.2958f;

  // Filtre complémentaire α = 0.98
  const float ALPHA = 0.98f;
  roll  = ALPHA * (roll  + gx * dt) + (1.0f - ALPHA) * rollAcc;
  pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * pitchAcc;
}

float pidStep(Pid &p, float setpoint, float measured, float dt){
  float err = setpoint - measured;
  p.integ += err * dt;
  // Anti-windup
  if(p.integ * p.ki > p.outMax) p.integ = p.outMax / (p.ki + 1e-6f);
  if(p.integ * p.ki < p.outMin) p.integ = p.outMin / (p.ki + 1e-6f);
  float deriv = (err - p.prevErr) / dt;
  p.prevErr = err;
  float out = p.kp*err + p.ki*p.integ + p.kd*deriv;
  if(out > p.outMax) out = p.outMax;
  if(out < p.outMin) out = p.outMin;
  return out;
}

void mixAndApply(float corrRoll, float corrPitch, float corrYaw){
  // Mixage quadricoptère configuration X (table 8.2 du rapport)
  //   M1 avant-gauche  CCW : THR - R + P
  //   M2 avant-droit   CW  : THR + R + P
  //   M3 arrière-gauche CW : THR - R - P
  //   M4 arrière-droit CCW : THR + R - P
  // Le lacet (yaw) joue sur le différentiel CW/CCW : +Y favorise les CW
  int m1 = throttleUs - (int)corrRoll + (int)corrPitch - (int)corrYaw;
  int m2 = throttleUs + (int)corrRoll + (int)corrPitch + (int)corrYaw;
  int m3 = throttleUs - (int)corrRoll - (int)corrPitch + (int)corrYaw;
  int m4 = throttleUs + (int)corrRoll - (int)corrPitch - (int)corrYaw;

  if(!armed){ m1 = m2 = m3 = m4 = ESC_US_MIN; }

  m1 = constrain(m1, ESC_US_MIN, ESC_US_MAX);
  m2 = constrain(m2, ESC_US_MIN, ESC_US_MAX);
  m3 = constrain(m3, ESC_US_MIN, ESC_US_MAX);
  m4 = constrain(m4, ESC_US_MIN, ESC_US_MAX);

  ledcWrite(CH_M1, usToDuty(m1));
  ledcWrite(CH_M2, usToDuty(m2));
  ledcWrite(CH_M3, usToDuty(m3));
  ledcWrite(CH_M4, usToDuty(m4));
}

// =====================================================================
//   TÉLÉMÉTRIE Serial — format parsé par la plateforme MediFly
// =====================================================================
void sendTelemetry(){
  float alt   = bmp.readAltitude(baroSeaLevelhPa) - altitudeOffset;
  float tempC = bmp.readTemperature();

  Serial.print(armed ? F("ARME  ") : F("DESARME "));
  Serial.print(F("| R:")); Serial.print(roll, 1);  Serial.print(F("\xB0"));
  Serial.print(F(" P:"));  Serial.print(pitch, 1); Serial.print(F("\xB0"));
  Serial.print(F(" | ALT:")); Serial.print(alt, 2); Serial.print(F("m"));
  Serial.print(F(" T:"));     Serial.print(tempC, 1); Serial.print(F("\xB0""C"));

  // GPS — toujours envoyer LAT/LNG (parsé par le site web)
  Serial.print(F(" | GPS:"));
  if(gps.location.isValid()){
    Serial.print(F("FIX"));
    Serial.print(F(" SAT:")); Serial.print(gps.satellites.value());
    Serial.print(F(" LAT:")); Serial.print(gps.location.lat(), 6);
    Serial.print(F(" LNG:")); Serial.print(gps.location.lng(), 6);
    if(gps.course.isValid()){ Serial.print(F(" HDG:")); Serial.print(gps.course.deg(), 1); }
    if(gps.speed.isValid()) { Serial.print(F(" SPD:")); Serial.print(gps.speed.kmph(), 1); }
  } else {
    Serial.print(F("--- SAT:")); Serial.print(gps.satellites.isValid()?gps.satellites.value():0);
    Serial.print(F(" LAT:--- LNG:---"));
  }

  Serial.println();
}

// =====================================================================
//   SETUP / LOOP
// =====================================================================
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n[BOOT] MediFly firmware — Groupe PLBD 19"));

  // I2C : MPU6050 + BMP280 sur le même bus
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if(!mpu.begin(0x68)){
    Serial.println(F("[ERR] MPU6050 introuvable — vérifier I2C (SDA=25 SCL=26)"));
    while(true){ delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.println(F("[OK]  MPU6050"));

  if(!bmp.begin(0x76) && !bmp.begin(0x77)){
    Serial.println(F("[WARN] BMP280 introuvable — altitude/temp désactivées"));
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println(F("[OK]  BMP280"));
    // Tare de l'altitude au sol (le drone est posé au boot)
    float a = 0;
    for(int i=0;i<20;i++){ a += bmp.readAltitude(baroSeaLevelhPa); delay(15); }
    altitudeOffset = a / 20.0f;
    Serial.print(F("[OK]  Tare altitude : ")); Serial.println(altitudeOffset, 2);
  }

  calibrateGyro();

  // GPS NEO-6M sur UART2
  GPSSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println(F("[OK]  GPS NEO-6M @ 9600 baud (UART2)"));

  // PWM moteurs + servo (LEDC v2)
  ledcSetup(CH_M1, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(PIN_M1, CH_M1);
  ledcSetup(CH_M2, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(PIN_M2, CH_M2);
  ledcSetup(CH_M3, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(PIN_M3, CH_M3);
  ledcSetup(CH_M4, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(PIN_M4, CH_M4);
  ledcSetup(CH_SERVO, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(PIN_SERVO, CH_SERVO);

  // Armement des ESC : envoyer 1050 µs pendant ~2 s
  Serial.println(F("[ESC] Armement (1050 µs pendant 2 s)..."));
  ledcWrite(CH_M1, usToDuty(ESC_US_INITIAL));
  ledcWrite(CH_M2, usToDuty(ESC_US_INITIAL));
  ledcWrite(CH_M3, usToDuty(ESC_US_INITIAL));
  ledcWrite(CH_M4, usToDuty(ESC_US_INITIAL));
  ledcWrite(CH_SERVO, usToDuty(SERVO_US_UP));
  delay(2000);
  ledcWrite(CH_M1, usToDuty(ESC_US_MIN));
  ledcWrite(CH_M2, usToDuty(ESC_US_MIN));
  ledcWrite(CH_M3, usToDuty(ESC_US_MIN));
  ledcWrite(CH_M4, usToDuty(ESC_US_MIN));

  // Bluepad32 — Bluetooth manette PS4
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  Serial.println(F("[BT]  En attente d'une manette PS4 (maintenir SHARE+PS pour appairer)"));

  Serial.println(F("[BOOT] Prêt — armement = L1+R1 / désarmement = OPTIONS"));
}

void loop(){
  // 1) Bluepad32 — lire la manette
  BP32.update();
  if(myCtrl) processController(myCtrl);

  // 2) Failsafe : perte de Bluetooth -> coupure des gaz
  if(armed && (millis() - lastBtSeenMs > FAILSAFE_TIMEOUT_MS)){
    armed = false;
    Serial.println(F("[SAFE] Failsafe Bluetooth -> DÉSARMEMENT"));
  }

  // 3) Lecture IMU + filtre complémentaire
  updateAttitude();

  // 4) Boucle PID + mixage moteurs
  static unsigned long lastPidMicros = 0;
  unsigned long now = micros();
  float dt = (lastPidMicros==0) ? 0.005f : (now - lastPidMicros) / 1.0e6f;
  if(dt > 0.05f) dt = 0.005f;
  lastPidMicros = now;

  float corrR = pidStep(pidRoll,  setRoll,  roll,  dt);
  float corrP = pidStep(pidPitch, setPitch, pitch, dt);
  float corrY = pidStep(pidYaw,   setYaw,   0.0f,  dt);  // pas de mesure de yaw fiable -> consigne directe
  mixAndApply(corrR, corrP, corrY);

  // 5) GPS — nourrir le parser TinyGPS++
  while(GPSSerial.available() > 0){ gps.encode(GPSSerial.read()); }

  // 6) Télémétrie périodique
  if(millis() - lastTelemMs >= TELEM_PERIOD_MS){
    lastTelemMs = millis();
    sendTelemetry();
  }
}
