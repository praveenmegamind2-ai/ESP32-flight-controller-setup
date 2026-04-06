/*
  ESP32 VTOL/Quad Hover Controller (PPM TX -> ESC outputs)

  +++++ Added: "Option A" Drift-Brake / Leaky Velocity Hold +++++
  - Uses MPU6050 DMP linear acceleration (gravity removed) transformed into WORLD frame.
  - Integrates to a leaky horizontal velocity estimate (vx, vy).
  - Applies a small PI "velocity hold" that nudges target pitch/roll angles to cancel drift.
  - Gated to avoid runaway: only active when armed, sticks centered, low rotation, throttle near hover.

  UI commands added:
    VHOLD,0/1
    VHPID,kp,ki,leak,maxDeg
    GETVHPID
    LEVELSET        (sets level reference offsets based on current roll/pitch)
    GETLEVEL
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Preferences.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Adafruit_BMP085.h> // BMP180/BMP085

/* =========================
WiFi AP + WebSocket
========================= */
const char* AP_SSID = "ESP32_PID";
const char* AP_PASS = "12345678";
WebSocketsServer ws(81);

/* =========================
NVS (Preferences)
========================= */
Preferences prefs;
static constexpr const char* NVS_NS_PID   = "pidstore";
static constexpr const char* NVS_NS_GYRO  = "gyroofs";
static constexpr const char* NVS_NS_TRIM  = "mottrim";
static constexpr const char* NVS_NS_VHOLD = "vhold";
static constexpr const char* NVS_NS_LEVEL = "level";

static inline void loadPIDFromNVS();
static inline void savePIDToNVS();
static inline void loadGyroOffsetsFromNVS();
static inline void saveGyroOffsetsToNVS(int16_t xOfs, int16_t yOfs, int16_t zOfs);
static inline void loadTrimFromNVS();
static inline void saveTrimToNVS();

/* =========================
I2C
========================= */
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;

/* =========================
Timing
========================= */
static constexpr uint32_t LOOP_HZ = 200;
static constexpr uint32_t LOOP_US = 1000000UL / LOOP_HZ;
uint32_t last_us = 0;

/* =========================
MPU6050 DMP
========================= */
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3]; // [yaw, pitch, roll] radians

// For drift-brake (DMP accel)
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;

/* =========================
Attitude
========================= */
bool att_init=false;
float roll=0, pitch=0, yaw=0;
float roll_rate=0, pitch_rate=0, yaw_rate=0;

/* =========================
Rate PID gains (UI tunes these)
========================= */
volatile float rKp=0.10f, rKi=0.00f, rKd=0.0020f;
volatile float pKp=0.10f, pKi=0.00f, pKd=0.0020f;
float i_roll_rate=0, i_pitch_rate=0;
static constexpr float I_CLAMP = 200.0f;

/* =========================
Yaw PID (rate) - now full PID
========================= */
volatile float yKp = 0.12f;
volatile float yKi = 0.03f;
volatile float yKd = 0.00f;

float yI = 0.0f;
float prev_yErr = 0.0f;
float yOut = 0.0f;
static constexpr float YI_CLAMP = 200.0f;

/* =========================
Motor trims (µs)  (persisted)
Positive trim = add µs to that motor
========================= */
int16_t tM1=0, tM2=0, tM3=0, tM4=0;

/* =========================
BMP180 (barometer)
========================= */
Adafruit_BMP085 bmp;
bool baro_ok=false;
float baro_tempC=0, baro_pressPa=0;
float baro_altM=0, baro_altFilt=0;
bool alt0_set=false;
float alt0=0;
static constexpr float BARO_LP = 0.90f;

/* =========================
PPM (Receiver)
========================= */
static constexpr int PPM_PIN = 26;
static constexpr int MAX_CH  = 8;

volatile uint16_t ppm[MAX_CH] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile uint8_t ppm_idx = 0;
volatile uint32_t ppm_lastMicros = 0;
volatile uint32_t ppm_lastFrameMicros = 0;

void IRAM_ATTR ppmISR() {
  uint32_t now = micros();
  uint32_t dt  = now - ppm_lastMicros;
  ppm_lastMicros = now;

  if (dt > 3000) {
    ppm_idx = 0;
    ppm_lastFrameMicros = now;
  } else if (ppm_idx < MAX_CH) {
    if (dt < 800)  dt = 800;
    if (dt > 2200) dt = 2200;
    ppm[ppm_idx++] = (uint16_t)dt;
  }
}
static inline bool ppmAlive(){
  uint32_t now = micros();
  return (now - ppm_lastFrameMicros) < 100000UL; // 100ms
}

/* =========================
Channels
========================= */
static constexpr int CH_ROLL  = 0;
static constexpr int CH_PITCH = 1;
static constexpr int CH_THR   = 2;
static constexpr int CH_YAW   = 3;
static constexpr int CH_ARM   = 4;

/* =========================
Helpers
========================= */
static inline int clampi(int x, int lo, int hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}
static inline float clampf(float x, float lo, float hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}
static inline float wrapPi(float a){
  while(a > 3.14159265f) a -= 6.28318531f;
  while(a < -3.14159265f) a += 6.28318531f;
  return a;
}

/* =========================
Throttle fix (your mapping preserved)
========================= */
static constexpr int THR_RAW_MIN = 1800;
static constexpr int THR_RAW_MAX = 2000;

static inline uint16_t throttleFixedUs(uint16_t thrRaw){
  int us = clampi((int)thrRaw, THR_RAW_MIN, THR_RAW_MAX);
  const int span = (THR_RAW_MAX - THR_RAW_MIN); // 200
  int mapped = 1000 + ((THR_RAW_MAX - us) * 1000) / span;

  if (mapped <= 1020) return 1000;
  if (mapped >= 1900) return 2000;
  return (uint16_t)clampi(mapped, 1000, 2000);
}

/* =========================
ESC OUTPUTS (LEDC new API)
========================= */
static constexpr int M1_PIN = 25;
static constexpr int M2_PIN = 13;
static constexpr int M3_PIN = 14;
static constexpr int M4_PIN = 27;

static constexpr uint32_t ESC_HZ = 250;
static constexpr uint8_t  ESC_RES_BITS = 16;

static inline void escAttachPin(int pin){
  ledcAttach(pin, ESC_HZ, ESC_RES_BITS);
}
static inline void escWriteUsPin(int pin, uint16_t us){
  us = (uint16_t)clampi(us, 1000, 2000);

  const uint32_t period_us = 1000000UL / ESC_HZ;
  const uint32_t maxDuty = (1UL << ESC_RES_BITS) - 1UL;

  uint32_t duty = (uint32_t)(( (uint64_t)us * maxDuty ) / period_us);
  if(duty > maxDuty) duty = maxDuty;
  ledcWrite(pin, duty);
}

/* =========================
Control params
========================= */
static constexpr float RAD2DEG = 57.29577951308232f;
static constexpr float DEG2RAD = 0.0174532925199433f;

static constexpr float MAX_ANGLE_DEG = 20.0f;
static constexpr float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG2RAD;

// Outer loop (self-level) -> desired rate
static constexpr float LEVEL_KP = 4.5f;            // deg/s per deg
static constexpr float MAX_RATE_CMD_DPS = 180.0f;  // limit outer output

static constexpr float MAX_YAW_RATE_DPS = 140.0f;

// Sticks
static constexpr float STICK_ROLL_SIGN  = +1.0f;
static constexpr float STICK_PITCH_SIGN = +1.0f;
static constexpr float STICK_YAW_SIGN   = +1.0f;

// IMU signs
static constexpr float IMU_ROLL_SIGN  = -1.0f;
static constexpr float IMU_PITCH_SIGN = -1.0f;
static constexpr float IMU_YAW_SIGN   = +1.0f;

// Mixer scaling
static constexpr float US_PER_UNIT = 2.0f;
static constexpr float AXIS_US_CLAMP = 250.0f;

static constexpr uint16_t IDLE_US = 1050;
static constexpr float    YAW_MIX_SIGN = +1.0f;

static constexpr uint16_t ARM_ON_US  = 1500;
static constexpr uint16_t THR_LOW_US = 1050;

/* =========================
Stick deadband (IMPORTANT)
========================= */
static constexpr int STICK_DEADBAND_US = 12; // 8-20 is typical

static inline int applyDeadband1500(int us){
  int d = us - 1500;
  if(abs(d) <= STICK_DEADBAND_US) return 1500;
  return us;
}

/* =========================
LEVEL reference offsets (deg) (persisted)
This fixes "my hover drifts because level is slightly off"
========================= */
volatile float levelRollBiasDeg  = 0.0f;
volatile float levelPitchBiasDeg = 0.0f;

static inline void loadLevelFromNVS(){
  prefs.begin(NVS_NS_LEVEL, true);
  bool has = prefs.getBool("has", false);
  if(has){
    levelRollBiasDeg  = prefs.getFloat("lr", 0.0f);
    levelPitchBiasDeg = prefs.getFloat("lp", 0.0f);
  }
  prefs.end();
}
static inline void saveLevelToNVS(){
  prefs.begin(NVS_NS_LEVEL, false);
  prefs.putBool("has", true);
  prefs.putFloat("lr", levelRollBiasDeg);
  prefs.putFloat("lp", levelPitchBiasDeg);
  prefs.end();
}

/* =========================
Velocity Hold / Drift-Brake (Option A) (persisted)
- Uses DMP world linear accel -> leaky velocity estimate (vx, vy)
- PI -> adds small angle corrections to target angles
========================= */
volatile bool vholdEnable = true;

// Tune defaults (safe starting point)
volatile float vKp = 0.70f;      // angle_deg per (m/s) error
volatile float vKi = 0.25f;      // angle_deg per (m/s*s) integral
volatile float vLeak = 1.2f;     // 1/s, higher = velocity decays faster
volatile float vMaxDeg = 2.0f;   // maximum correction angle (deg)

static constexpr float G_TO_MS2 = 9.80665f;
static constexpr float ACC_COUNTS_PER_G = 16384.0f;

// State
float vx=0, vy=0;        // leaky velocity estimate (m/s)
float vIx=0, vIy=0;      // integrators for velocity error -> angle deg
static constexpr float V_I_CLAMP = 6.0f; // deg integrator clamp

// Gating thresholds
static constexpr float VHOLD_MAX_RATE_DPS = 50.0f; // if rotating too much, disable
static constexpr uint16_t VHOLD_THR_MIN_US = 1150; // near hover-ish (adjust if needed)
static constexpr float VHOLD_A_CLAMP = 6.0f; // m/s^2 clamp for safety

// If drift correction acts backwards, flip these:
static constexpr float VHOLD_PITCH_SIGN = +1.0f; // affects target pitch (forward/back)
static constexpr float VHOLD_ROLL_SIGN  = +1.0f; // affects target roll (left/right)

static inline void loadVholdFromNVS(){
  prefs.begin(NVS_NS_VHOLD, true);
  bool has = prefs.getBool("has", false);
  if(has){
    vholdEnable = prefs.getBool("en", vholdEnable);
    vKp   = prefs.getFloat("kp", vKp);
    vKi   = prefs.getFloat("ki", vKi);
    vLeak = prefs.getFloat("lk", vLeak);
    vMaxDeg = prefs.getFloat("mx", vMaxDeg);
  }
  prefs.end();
}
static inline void saveVholdToNVS(){
  prefs.begin(NVS_NS_VHOLD, false);
  prefs.putBool("has", true);
  prefs.putBool("en", vholdEnable);
  prefs.putFloat("kp", vKp);
  prefs.putFloat("ki", vKi);
  prefs.putFloat("lk", vLeak);
  prefs.putFloat("mx", vMaxDeg);
  prefs.end();
}

/* =========================
Motor mixing (YOUR X layout) + yaw grouping
M1: Front Right
M2: Rear  Left
M3: Front Left
M4: Rear  Right

Yaw grouping:
CCW: M1, M2  -> +yaw_us
CW : M3, M4  -> -yaw_us
========================= */
static inline void mixMotors_XLayout(
  uint16_t thr,
  float roll_us,
  float pitch_us,
  float yaw_us,
  uint16_t &m1,
  uint16_t &m2,
  uint16_t &m3,
  uint16_t &m4
){
  float t = (float)thr;

  float M1 = t + (-pitch_us) + (-roll_us) + (+yaw_us); // Front Right
  float M2 = t + (+pitch_us) + (+roll_us) + (+yaw_us); // Rear  Left
  float M3 = t + (-pitch_us) + (+roll_us) - (yaw_us);  // Front Left
  float M4 = t + (+pitch_us) + (-roll_us) - (yaw_us);  // Rear  Right

  m1 = (uint16_t)clampi((int)lroundf(M1), 1000, 2000);
  m2 = (uint16_t)clampi((int)lroundf(M2), 1000, 2000);
  m3 = (uint16_t)clampi((int)lroundf(M3), 1000, 2000);
  m4 = (uint16_t)clampi((int)lroundf(M4), 1000, 2000);
}

/* Apply trims with headroom protection */
static inline uint16_t applyTrim(uint16_t base, int16_t trim, int16_t maxPosTrim){
  int limitedBase = (int)base;
  int maxBase = 2000 - (int)maxPosTrim;
  if(maxBase < 1000) maxBase = 1000;
  if(limitedBase > maxBase) limitedBase = maxBase;

  int out = limitedBase + (int)trim;
  return (uint16_t)clampi(out, 1000, 2000);
}

/* =========================
Telemetry structs (unchanged)
========================= */
#pragma pack(push,1)
struct TelemetryV2 {
  uint16_t magic;
  uint8_t ver;
  uint8_t type;
  uint32_t t_us;

  float roll,pitch,yaw;
  float roll_rate,pitch_rate,yaw_rate;

  float rP,rI,rD,rOut;
  float pP,pI,pD,pOut;

  float magHeading;
  uint8_t magType;
  uint8_t pad[3];
};
#pragma pack(pop)

#pragma pack(push,1)
struct BaroV1 {
  uint16_t magic;
  uint8_t ver;
  uint8_t type;
  uint32_t t_us;

  float tempC;
  float pressPa;
  float altM;
  float altFiltM;
  uint8_t ok;
  uint8_t pad[3];
};
#pragma pack(pop)

#pragma pack(push,1)
struct RCMotV1 {
  uint16_t magic;
  uint8_t ver;
  uint8_t type;
  uint32_t t_us;

  uint16_t ch[MAX_CH];
  uint16_t m[4];
  uint8_t isArmed;
  uint8_t pad[3];
};
#pragma pack(pop)

/* =========================
Subscriptions
========================= */
volatile bool sub_baro = true;
volatile bool sub_rc   = false;

/* =========================
NVS PID helpers
========================= */
static inline void loadPIDFromNVS(){
  prefs.begin(NVS_NS_PID, true);
  bool has = prefs.getBool("has", false);
  if(has){
    rKp = prefs.getFloat("rKp", rKp);
    rKi = prefs.getFloat("rKi", rKi);
    rKd = prefs.getFloat("rKd", rKd);
    pKp = prefs.getFloat("pKp", pKp);
    pKi = prefs.getFloat("pKi", pKi);
    pKd = prefs.getFloat("pKd", pKd);

    yKp = prefs.getFloat("yKp", yKp);
    yKi = prefs.getFloat("yKi", yKi);
    yKd = prefs.getFloat("yKd", yKd);
  }
  prefs.end();
}
static inline void savePIDToNVS(){
  prefs.begin(NVS_NS_PID, false);
  prefs.putBool("has", true);
  prefs.putFloat("rKp", rKp);
  prefs.putFloat("rKi", rKi);
  prefs.putFloat("rKd", rKd);
  prefs.putFloat("pKp", pKp);
  prefs.putFloat("pKi", pKi);
  prefs.putFloat("pKd", pKd);

  prefs.putFloat("yKp", yKp);
  prefs.putFloat("yKi", yKi);
  prefs.putFloat("yKd", yKd);
  prefs.end();
}

/* =========================
NVS gyro offset helpers
========================= */
static inline void loadGyroOffsetsFromNVS(){
  prefs.begin(NVS_NS_GYRO, true);
  bool has = prefs.getBool("has", false);
  if(has){
    int16_t xo = (int16_t)prefs.getShort("xo", 0);
    int16_t yo = (int16_t)prefs.getShort("yo", 0);
    int16_t zo = (int16_t)prefs.getShort("zo", 0);
    mpu.setXGyroOffset(xo);
    mpu.setYGyroOffset(yo);
    mpu.setZGyroOffset(zo);
  }
  prefs.end();
}
static inline void saveGyroOffsetsToNVS(int16_t xOfs, int16_t yOfs, int16_t zOfs){
  prefs.begin(NVS_NS_GYRO, false);
  prefs.putBool("has", true);
  prefs.putShort("xo", xOfs);
  prefs.putShort("yo", yOfs);
  prefs.putShort("zo", zOfs);
  prefs.end();
}

/* =========================
NVS trim helpers
========================= */
static inline void loadTrimFromNVS(){
  prefs.begin(NVS_NS_TRIM, true);
  bool has = prefs.getBool("has", false);
  if(has){
    tM1 = (int16_t)prefs.getShort("t1", 0);
    tM2 = (int16_t)prefs.getShort("t2", 0);
    tM3 = (int16_t)prefs.getShort("t3", 0);
    tM4 = (int16_t)prefs.getShort("t4", 0);
  }
  prefs.end();
}
static inline void saveTrimToNVS(){
  prefs.begin(NVS_NS_TRIM, false);
  prefs.putBool("has", true);
  prefs.putShort("t1", tM1);
  prefs.putShort("t2", tM2);
  prefs.putShort("t3", tM3);
  prefs.putShort("t4", tM4);
  prefs.end();
}

/* =========================
Gyro calibration (unchanged)
========================= */
volatile bool gyroCalReq = false;
bool gyroCalRunning = false;

static constexpr uint32_t GYROCAL_MIN_MS = 2000;
static constexpr uint32_t GYROCAL_MAX_MS = 8000;
static constexpr uint16_t GYROCAL_RATE_HZ = 400;
static constexpr uint32_t GYROCAL_DT_US = 1000000UL / GYROCAL_RATE_HZ;

static constexpr int16_t GYRO_STILL_ABS_MAX = 300;
static constexpr float   GYRO_STILL_GOOD_FRAC = 0.90f;

static inline void startGyroCal(){
  gyroCalReq = false;
  gyroCalRunning = true;
  ws.broadcastTXT("GYROCAL,START");
}
static inline void finishGyroCal(bool ok, const char* why, int16_t xo, int16_t yo, int16_t zo){
  gyroCalRunning = false;

  if(ok){
    mpu.setXGyroOffset(xo);
    mpu.setYGyroOffset(yo);
    mpu.setZGyroOffset(zo);
    saveGyroOffsetsToNVS(xo,yo,zo);

    mpu.setDMPEnabled(false);
    mpu.resetFIFO();
    mpu.setDMPEnabled(true);

    att_init = false;
    i_roll_rate = 0; i_pitch_rate = 0;

    // also reset drift-brake states
    vx=vy=0; vIx=vIy=0;

    char msg[96];
    snprintf(msg, sizeof(msg), "GYROCAL,OK,xo=%d,yo=%d,zo=%d", xo,yo,zo);
    ws.broadcastTXT(msg);
  } else {
    char msg[96];
    snprintf(msg, sizeof(msg), "GYROCAL,FAIL,%s", why ? why : "unknown");
    ws.broadcastTXT(msg);
  }
}
static inline void runGyroCal(){
  escWriteUsPin(M1_PIN, 1000);
  escWriteUsPin(M2_PIN, 1000);
  escWriteUsPin(M3_PIN, 1000);
  escWriteUsPin(M4_PIN, 1000);

  const uint32_t startMs = millis();
  uint32_t lastSampleUs = micros();

  int64_t sx=0, sy=0, sz=0;
  uint32_t n=0, good=0;

  while(true){
    ws.loop();

    const uint32_t nowMs = millis();
    if(nowMs - startMs > GYROCAL_MAX_MS){
      finishGyroCal(false, "timeout_move", 0,0,0);
      return;
    }

    const uint32_t nowUs = micros();
    if(nowUs - lastSampleUs < GYROCAL_DT_US) continue;
    lastSampleUs = nowUs;

    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    const bool still = (abs(gx) <= GYRO_STILL_ABS_MAX) &&
                       (abs(gy) <= GYRO_STILL_ABS_MAX) &&
                       (abs(gz) <= GYRO_STILL_ABS_MAX);

    sx += gx; sy += gy; sz += gz;
    n++;
    if(still) good++;

    if((nowMs - startMs) >= GYROCAL_MIN_MS){
      const float frac = (n > 0) ? ((float)good / (float)n) : 0.0f;
      if(frac >= GYRO_STILL_GOOD_FRAC){
        const float mx = (float)sx / (float)n;
        const float my = (float)sy / (float)n;
        const float mz = (float)sz / (float)n;

        int16_t xo = (int16_t)lroundf(-mx);
        int16_t yo = (int16_t)lroundf(-my);
        int16_t zo = (int16_t)lroundf(-mz);

        finishGyroCal(true, nullptr, xo, yo, zo);
        return;
      }
    }
  }
}

/* =========================
WebSocket helpers
========================= */
static inline void sendPIDCUR(uint8_t num){
  char msg[220];
  snprintf(msg, sizeof(msg),
    "PIDCUR,rKp=%.4f,rKi=%.4f,rKd=%.4f,pKp=%.4f,pKi=%.4f,pKd=%.4f,yKp=%.4f,yKi=%.4f,yKd=%.4f",
    rKp,rKi,rKd,pKp,pKi,pKd,yKp,yKi,yKd
  );
  ws.sendTXT(num, msg);
}
static inline void sendTRIMCUR(uint8_t num){
  char msg[120];
  snprintf(msg, sizeof(msg),
    "TRIMCUR,t1=%d,t2=%d,t3=%d,t4=%d",
    (int)tM1,(int)tM2,(int)tM3,(int)tM4
  );
  ws.sendTXT(num, msg);
}
static inline void sendVHPIDCUR(uint8_t num){
  char msg[160];
  snprintf(msg, sizeof(msg),
    "VHPIDCUR,en=%d,kp=%.3f,ki=%.3f,leak=%.3f,maxDeg=%.2f",
    vholdEnable?1:0, vKp, vKi, vLeak, vMaxDeg
  );
  ws.sendTXT(num, msg);
}
static inline void sendLEVELCUR(uint8_t num){
  char msg[120];
  snprintf(msg, sizeof(msg),
    "LEVELCUR,lr=%.2f,lp=%.2f",
    levelRollBiasDeg, levelPitchBiasDeg
  );
  ws.sendTXT(num, msg);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len){
  if(type == WStype_CONNECTED){
    ws.sendTXT(num, "PIDACK,CONNECTED");
    sendPIDCUR(num);
    sendTRIMCUR(num);
    sendVHPIDCUR(num);
    sendLEVELCUR(num);

    char submsg[64];
    snprintf(submsg, sizeof(submsg), "SUBACK,BARO,%d,RC,%d", sub_baro?1:0, sub_rc?1:0);
    ws.sendTXT(num, submsg);
    return;
  }
  if(type != WStype_TEXT) return;

  static char buf[240];
  size_t n = (len < sizeof(buf)-1) ? len : (sizeof(buf)-1);
  memcpy(buf, payload, n);
  buf[n] = '\0';

  // ----- PID set -----
  float a,b,c,d,e,f;
  if(sscanf(buf, "PID,%f,%f,%f,%f,%f,%f", &a,&b,&c,&d,&e,&f) == 6){
    rKp=a; rKi=b; rKd=c;
    pKp=d; pKi=e; pKd=f;
    savePIDToNVS();
    ws.sendTXT(num, "PIDACK,OK_SAVED");
    sendPIDCUR(num);
    return;
  }

  // ----- Yaw PID set -----
  float ykp,yki,ykd;
  if(sscanf(buf, "YAWPID,%f,%f,%f", &ykp,&yki,&ykd) == 3){
    yKp=ykp; yKi=yki; yKd=ykd;
    savePIDToNVS();
    ws.sendTXT(num, "YAWPIDACK,OK_SAVED");
    sendPIDCUR(num);
    return;
  }
  if(strcmp(buf, "GETYAWPID") == 0){
    // reuse PIDCUR line since it contains yaw too
    sendPIDCUR(num);
    return;
  }

  // ----- GETPID -----
  if(strcmp(buf, "GETPID") == 0){
    sendPIDCUR(num);
    return;
  }

  // ----- TRIM set -----
  int t1,t2,t3,t4;
  if(sscanf(buf, "TRIM,%d,%d,%d,%d", &t1,&t2,&t3,&t4) == 4 ||
     sscanf(buf, "MTRIM,%d,%d,%d,%d", &t1,&t2,&t3,&t4) == 4){
    tM1 = (int16_t)clampi(t1, -300, 300);
    tM2 = (int16_t)clampi(t2, -300, 300);
    tM3 = (int16_t)clampi(t3, -300, 300);
    tM4 = (int16_t)clampi(t4, -300, 300);
    saveTrimToNVS();
    ws.sendTXT(num, "TRIMACK,OK_SAVED");
    sendTRIMCUR(num);
    return;
  }
  if(strcmp(buf, "GETTRIM") == 0){
    sendTRIMCUR(num);
    return;
  }

  // ----- Gyro calibration -----
  if(strcmp(buf, "GYROCAL") == 0 || strcmp(buf, "GYRO,CAL") == 0){
    gyroCalReq = true;
    ws.sendTXT(num, "GYROCAL,QUEUED");
    return;
  }

  // ----- Velocity Hold enable -----
  int en;
  if(sscanf(buf, "VHOLD,%d", &en) == 1){
    vholdEnable = (en != 0);
    saveVholdToNVS();
    ws.sendTXT(num, vholdEnable ? "VHOLDACK,EN=1" : "VHOLDACK,EN=0");
    sendVHPIDCUR(num);
    return;
  }

  // ----- Velocity Hold PID params -----
  float kp,ki,lk,mxdeg;
  if(sscanf(buf, "VHPID,%f,%f,%f,%f", &kp,&ki,&lk,&mxdeg) == 4){
    vKp   = clampf(kp,   0.0f, 5.0f);
    vKi   = clampf(ki,   0.0f, 5.0f);
    vLeak = clampf(lk,   0.0f, 5.0f);
    vMaxDeg = clampf(mxdeg, 0.0f, 6.0f);
    saveVholdToNVS();
    ws.sendTXT(num, "VHPIDACK,OK_SAVED");
    sendVHPIDCUR(num);
    return;
  }
  if(strcmp(buf, "GETVHPID") == 0){
    sendVHPIDCUR(num);
    return;
  }

  // ----- Level set -----
  if(strcmp(buf, "LEVELSET") == 0){
    // Use current attitude as "level"
    // (only valid if attitude is initialized)
    if(att_init){
      levelRollBiasDeg  = roll * RAD2DEG;
      levelPitchBiasDeg = pitch * RAD2DEG;
      saveLevelToNVS();

      // reset drift-brake states
      vx=vy=0; vIx=vIy=0;

      ws.sendTXT(num, "LEVELACK,OK_SAVED");
      sendLEVELCUR(num);
    } else {
      ws.sendTXT(num, "LEVELACK,FAIL_NOATT");
    }
    return;
  }
  if(strcmp(buf, "GETLEVEL") == 0){
    sendLEVELCUR(num);
    return;
  }

  // ----- Subscriptions -----
  char which[16];
  int onoff;
  if(sscanf(buf, "SUB,%15[^,],%d", which, &onoff) == 2){
    bool enb = (onoff != 0);
    if(strcmp(which, "BARO") == 0) sub_baro = enb;
    if(strcmp(which, "RC")   == 0) sub_rc   = enb;

    char msg2[64];
    snprintf(msg2, sizeof(msg2), "SUBACK,BARO,%d,RC,%d", sub_baro?1:0, sub_rc?1:0);
    ws.sendTXT(num, msg2);
    return;
  }
}

/* =========================
Setup
========================= */
void setup(){
  // Load persistent settings first
  loadPIDFromNVS();
  loadTrimFromNVS();
  loadVholdFromNVS();
  loadLevelFromNVS();

  // PPM
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);

  // ESC outputs
  escAttachPin(M1_PIN);
  escAttachPin(M2_PIN);
  escAttachPin(M3_PIN);
  escAttachPin(M4_PIN);
  escWriteUsPin(M1_PIN, 1000);
  escWriteUsPin(M2_PIN, 1000);
  escWriteUsPin(M3_PIN, 1000);
  escWriteUsPin(M4_PIN, 1000);

  // WiFi + WS
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  ws.begin();
  ws.onEvent(onWsEvent);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // MPU init
  mpu.initialize();
  loadGyroOffsetsFromNVS(); // apply saved offsets before DMP init

  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
    mpu.setDMPEnabled(true);
    dmpReady = true;
  }

  // Barometer
  baro_ok = bmp.begin(BMP085_ULTRAHIGHRES, &Wire);

  last_us = micros();
}

/* =========================
Loop
========================= */
void loop(){
  ws.loop();
  if(!dmpReady) return;

  // Handle queued gyro calibration
  if(gyroCalReq && !gyroCalRunning){
    startGyroCal();
    runGyroCal();
  }

  const uint32_t now = micros();
  if(now - last_us < LOOP_US) return;
  const float dt = (now - last_us) * 1e-6f;
  last_us = now;

  /* -------- RC snapshot -------- */
  uint16_t ch[MAX_CH];
  noInterrupts();
  for(int i=0;i<MAX_CH;i++) ch[i]=ppm[i];
  interrupts();

  // deadband roll/pitch/yaw around 1500
  ch[CH_ROLL]  = (uint16_t)applyDeadband1500(ch[CH_ROLL]);
  ch[CH_PITCH] = (uint16_t)applyDeadband1500(ch[CH_PITCH]);
  ch[CH_YAW]   = (uint16_t)applyDeadband1500(ch[CH_YAW]);

  ch[CH_THR] = throttleFixedUs(ch[CH_THR]);
  const bool rx_ok = ppmAlive();

  /* -------- TX arming (latch) -------- */
  static bool armed_latched = false;

  if(!rx_ok){
    armed_latched = false;
  } else {
    const bool armSwitch = (ch[CH_ARM] > ARM_ON_US);
    const bool thrLow = (ch[CH_THR] <= THR_LOW_US);

    if(!armed_latched){
      if(armSwitch && thrLow) armed_latched = true;
    } else {
      if(!armSwitch) armed_latched = false;
    }
  }
  bool isArmed = armed_latched;
  if(gyroCalRunning) isArmed = false;

  /* -------- Attitude + gyro rates + accel -------- */
  bool gotPacket = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  if(gotPacket){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // DMP output order: ypr[0]=yaw, ypr[1]=pitch, ypr[2]=roll
    // Keep your swap (matching your UI)
    float y = IMU_YAW_SIGN   * ypr[0];
    float p = IMU_PITCH_SIGN * ypr[2];
    float r = IMU_ROLL_SIGN  * ypr[1];

    yaw   = wrapPi(y);
    pitch = wrapPi(p);
    roll  = wrapPi(r);

    // Raw gyro (deg/s) for rate PID
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz); // raw counts
    roll_rate  = IMU_ROLL_SIGN  * ((float)gx / 131.0f);
    pitch_rate = IMU_PITCH_SIGN * ((float)gy / 131.0f);
    yaw_rate   = IMU_YAW_SIGN   * ((float)gz / 131.0f);

    // DMP accel -> linear accel (gravity removed) -> world frame
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    att_init = true;
  }

  /* -------- Barometer -------- */
  if(sub_baro && baro_ok){
    baro_tempC  = bmp.readTemperature();
    baro_pressPa= (float)bmp.readPressure();
    baro_altM   = bmp.readAltitude();

    if(!alt0_set){
      alt0_set = true;
      alt0 = baro_altM;
      baro_altFilt = 0;
    } else {
      float rel = baro_altM - alt0;
      baro_altFilt = BARO_LP*baro_altFilt + (1.0f - BARO_LP)*rel;
    }
  }

  /* =========================
     CONTROL: compute PID ALWAYS (for graphs),
     but only APPLY to motors when ARMED.
  ========================= */
  uint16_t m1=1000,m2=1000,m3=1000,m4=1000;

  float rPterm=0,rIterm=0,rDterm=0,rOut=0;
  float pPterm=0,pIterm=0,pDterm=0,pOut=0;

  if(att_init){
    // Stick normalized
    float stickRoll  = ((int)ch[CH_ROLL]  - 1500) / 500.0f;
    float stickPitch = ((int)ch[CH_PITCH] - 1500) / 500.0f;
    float stickYaw   = ((int)ch[CH_YAW]   - 1500) / 500.0f;

    stickRoll  = clampf(stickRoll,  -1, 1) * STICK_ROLL_SIGN;
    stickPitch = clampf(stickPitch, -1, 1) * STICK_PITCH_SIGN;
    stickYaw   = clampf(stickYaw,   -1, 1) * STICK_YAW_SIGN;

    // Base target angles (rad)
    float tgtRollRad  = stickRoll  * MAX_ANGLE_RAD;
    float tgtPitchRad = stickPitch * MAX_ANGLE_RAD;

    // Measured angles (deg) corrected by level offsets
    float rollDegMeas  = (roll  * RAD2DEG) - levelRollBiasDeg;
    float pitchDegMeas = (pitch * RAD2DEG) - levelPitchBiasDeg;

    float tgtRollDeg  = tgtRollRad  * RAD2DEG;
    float tgtPitchDeg = tgtPitchRad * RAD2DEG;

    /* =========================
       Drift-Brake / Velocity Hold (Option A)
       - only when sticks near center
       - only when armed and throttle is in hover-ish region
       - only when rotation rate is low
    ========================= */
    const bool sticksCentered = (fabsf(stickRoll) < 0.06f) && (fabsf(stickPitch) < 0.06f) && (fabsf(stickYaw) < 0.10f);
    const bool lowRotation = (fabsf(roll_rate) < VHOLD_MAX_RATE_DPS) && (fabsf(pitch_rate) < VHOLD_MAX_RATE_DPS);
    const bool thrOk = (ch[CH_THR] >= VHOLD_THR_MIN_US);

    bool vholdActive = vholdEnable && isArmed && sticksCentered && lowRotation && thrOk && !gyroCalRunning;

    if(vholdActive){
      // WORLD linear accel (counts) -> m/s^2
      float ax = ( (float)aaWorld.x / ACC_COUNTS_PER_G ) * G_TO_MS2;
      float ay = ( (float)aaWorld.y / ACC_COUNTS_PER_G ) * G_TO_MS2;

      // Safety clamp (noise/vibration)
      ax = clampf(ax, -VHOLD_A_CLAMP, VHOLD_A_CLAMP);
      ay = clampf(ay, -VHOLD_A_CLAMP, VHOLD_A_CLAMP);

      // Leaky integrate to velocity estimate
      // vx,vy decay toward 0 over time with vLeak
      float leak = clampf(vLeak, 0.0f, 5.0f);
      float decay = expf(-leak * dt);       // stable decay
      vx = vx * decay + ax * dt;
      vy = vy * decay + ay * dt;

      // We want vx=0, vy=0
      float ex = -vx;
      float ey = -vy;

      // Integrate error -> angle correction (deg)
      vIx = clampf(vIx + ex * dt, -V_I_CLAMP, V_I_CLAMP);
      vIy = clampf(vIy + ey * dt, -V_I_CLAMP, V_I_CLAMP);

      float corrPitchDeg = (vKp * ex + vKi * vIx); // pitch correction
      float corrRollDeg  = (vKp * ey + vKi * vIy); // roll correction

      // Clamp correction angle (tiny!)
      corrPitchDeg = clampf(corrPitchDeg, -vMaxDeg, vMaxDeg);
      corrRollDeg  = clampf(corrRollDeg,  -vMaxDeg, vMaxDeg);

      // Apply to targets
      tgtPitchDeg += VHOLD_PITCH_SIGN * corrPitchDeg;
      tgtRollDeg  += VHOLD_ROLL_SIGN  * corrRollDeg;
    } else {
      // When vhold not active, gently reset states (prevents stale correction)
      vx *= 0.90f;
      vy *= 0.90f;
      vIx *= 0.90f;
      vIy *= 0.90f;
    }

    // Angle error (deg)
    float eRollDeg  = (tgtRollDeg  - rollDegMeas);
    float ePitchDeg = (tgtPitchDeg - pitchDegMeas);

    // Outer loop -> desired rates (deg/s)
    float desRollRate  = clampf(LEVEL_KP * eRollDeg,  -MAX_RATE_CMD_DPS, MAX_RATE_CMD_DPS);
    float desPitchRate = clampf(LEVEL_KP * ePitchDeg, -MAX_RATE_CMD_DPS, MAX_RATE_CMD_DPS);
    float desYawRate   = stickYaw * MAX_YAW_RATE_DPS;

    // Inner loop errors (rate)
    float eR = desRollRate  - roll_rate;
    float eP = desPitchRate - pitch_rate;
    float eY = desYawRate   - yaw_rate;

    // Integrate (roll/pitch)
    i_roll_rate  = clampf(i_roll_rate  + eR * dt, -I_CLAMP, I_CLAMP);
    i_pitch_rate = clampf(i_pitch_rate + eP * dt, -I_CLAMP, I_CLAMP);

    // Derivative of rate error (roll/pitch)
    static float prev_eR = 0, prev_eP = 0;
    float dR = (dt > 1e-6f) ? ((eR - prev_eR) / dt) : 0.0f;
    float dP = (dt > 1e-6f) ? ((eP - prev_eP) / dt) : 0.0f;
    prev_eR = eR;
    prev_eP = eP;

    // Roll/Pitch Rate PID terms
    rPterm = rKp * eR;
    rIterm = rKi * i_roll_rate;
    rDterm = rKd * dR;

    pPterm = pKp * eP;
    pIterm = pKi * i_pitch_rate;
    pDterm = pKd * dP;

    rOut = (rPterm + rIterm + rDterm) * US_PER_UNIT;
    pOut = (pPterm + pIterm + pDterm) * US_PER_UNIT;

    rOut = clampf(rOut, -AXIS_US_CLAMP, AXIS_US_CLAMP);
    pOut = clampf(pOut, -AXIS_US_CLAMP, AXIS_US_CLAMP);

    // Yaw PID (rate)
    yI = clampf(yI + eY * dt, -YI_CLAMP, YI_CLAMP);
    float dY = (dt > 1e-6f) ? ((eY - prev_yErr) / dt) : 0.0f;
    prev_yErr = eY;

    yOut = (yKp * eY + yKi * yI + yKd * dY) * US_PER_UNIT;
    yOut = clampf(yOut, -AXIS_US_CLAMP, AXIS_US_CLAMP);
    yOut *= YAW_MIX_SIGN;

    if(!isArmed){
      // prevent wind-up while disarmed
      i_roll_rate = 0;
      i_pitch_rate = 0;
      yI = 0;

      escWriteUsPin(M1_PIN, 1000);
      escWriteUsPin(M2_PIN, 1000);
      escWriteUsPin(M3_PIN, 1000);
      escWriteUsPin(M4_PIN, 1000);

      m1=m2=m3=m4=1000;
    } else {
      uint16_t thr = ch[CH_THR];
      if(thr < IDLE_US) thr = IDLE_US;

      uint16_t bm1,bm2,bm3,bm4;
      mixMotors_XLayout(thr, rOut, pOut, yOut, bm1,bm2,bm3,bm4);

      int16_t maxPosTrim = 0;
      if(tM1 > maxPosTrim) maxPosTrim = tM1;
      if(tM2 > maxPosTrim) maxPosTrim = tM2;
      if(tM3 > maxPosTrim) maxPosTrim = tM3;
      if(tM4 > maxPosTrim) maxPosTrim = tM4;
      if(maxPosTrim < 0) maxPosTrim = 0;

      m1 = applyTrim(bm1, tM1, maxPosTrim);
      m2 = applyTrim(bm2, tM2, maxPosTrim);
      m3 = applyTrim(bm3, tM3, maxPosTrim);
      m4 = applyTrim(bm4, tM4, maxPosTrim);

      escWriteUsPin(M1_PIN, m1);
      escWriteUsPin(M2_PIN, m2);
      escWriteUsPin(M3_PIN, m3);
      escWriteUsPin(M4_PIN, m4);
    }
  } else {
    i_roll_rate = 0;
    i_pitch_rate = 0;
    yI = 0;
    vx=vy=0; vIx=vIy=0;

    escWriteUsPin(M1_PIN, 1000);
    escWriteUsPin(M2_PIN, 1000);
    escWriteUsPin(M3_PIN, 1000);
    escWriteUsPin(M4_PIN, 1000);
    m1=m2=m3=m4=1000;
  }

  /* -------- Telemetry type 1 (always) -------- */
  TelemetryV2 t{};
  t.magic = 0xA55A;
  t.ver   = 2;
  t.type  = 1;
  t.t_us  = micros();

  t.roll = roll;
  t.pitch= pitch;
  t.yaw  = yaw;

  t.roll_rate  = roll_rate;
  t.pitch_rate = pitch_rate;
  t.yaw_rate   = yaw_rate;

  t.rP=rPterm; t.rI=rIterm; t.rD=rDterm; t.rOut=rOut;
  t.pP=pPterm; t.pI=pIterm; t.pD=pDterm; t.pOut=pOut;

  t.magHeading = 0;
  t.magType = 0;

  ws.broadcastBIN((uint8_t*)&t, sizeof(t));

  /* -------- Baro telemetry type 2 -------- */
  if(sub_baro){
    BaroV1 b{};
    b.magic=0xA55A; b.ver=1; b.type=2; b.t_us=micros();
    b.tempC = baro_tempC;
    b.pressPa = baro_pressPa;
    b.altM = alt0_set ? (baro_altM - alt0) : 0;
    b.altFiltM = baro_altFilt;
    b.ok = (uint8_t)(baro_ok ? 1 : 0);
    ws.broadcastBIN((uint8_t*)&b, sizeof(b));
  }

  /* -------- RC/Motor telemetry type 3 -------- */
  if(sub_rc){
    RCMotV1 r{};
    r.magic=0xA55A; r.ver=1; r.type=3; r.t_us=micros();
    for(int i=0;i<MAX_CH;i++) r.ch[i]=ch[i];
    r.m[0]=m1; r.m[1]=m2; r.m[2]=m3; r.m[3]=m4;
    r.isArmed = (uint8_t)(isArmed ? 1 : 0);
    ws.broadcastBIN((uint8_t*)&r, sizeof(r));
  }
}
