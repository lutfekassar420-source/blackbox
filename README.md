# blackbox
 الغرض الرئيسي من هذا المشروع هو تطوير نظام صندوق أسود للسالصندوق الأسود اللاسلكي هو في الأساس جهاز يوضح حالة وقوع حادث للسيارة بالصوت والصورة (فيديو) من كافة الاتجاهات (4 كاميرا)، كما يخزن ويعرض البارامترات كل ثلاث ثوانٍ مثل التاريخ والوقت ودرجة الحرارة والموقع والاهتزاز والحد الأقصى للكحول وسرعة السيارة مع منبه عند زيادة سرعة السيارة عن 120 كم/سا
/*
  ESP32-CAM (AI Thinker)
  - Receives telemetry lines over Serial (UART0)
  - Logs to SD_MMC: /car_log.txt
  - When sees "REC=s" => start frame capture to SD (10 FPS)
  - When sees "REC=e" => stop frame capture
  - Flash LED GPIO4 ON during recording

  Note:
  - Saves frames as JPG images in a folder (not AVI).
  - You can convert frames to video later on PC (ffmpeg).
*/

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// =================== Camera Pins (AI Thinker ESP32-CAM) ===================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Flash LED
const int FLASH_PIN = 4;

bool recording = false;
unsigned long frameTimer = 0;
const unsigned long FRAME_PERIOD_MS = 100; // 10 FPS

String sessionFolder = "";
uint32_t frameIndex = 0;

String lineBuffer = "";

// ===========================
// SD init
// ===========================
bool initSD() {
  if (!SD_MMC.begin("/sdcard", true)) { // 1-bit mode (more stable)
    return false;
  }
  return true;
}

// ===========================
// Camera init
// ===========================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  // settings
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;   // 640x480
  config.jpeg_quality = 12;             // 10-15 good
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  return (err == ESP_OK);
}

// ===========================
// Log telemetry line to SD
// ===========================
void logLineToSD(const String &line) {
  File f = SD_MMC.open("/car_log.txt", FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

// ===========================
// Create folder name
// ===========================
String makeSessionFolder() {
  // simple folder based on millis
  String name = "/REC_" + String((uint32_t)millis());
  return name;
}

// ===========================
// Start/Stop recording
// ===========================
void startRecording() {
  if (recording) return;

  sessionFolder = makeSessionFolder();
  SD_MMC.mkdir(sessionFolder);

  recording = true;
  frameIndex = 0;
  frameTimer = millis();

  digitalWrite(FLASH_PIN, HIGH);
}

void stopRecording() {
  if (!recording) return;

  recording = false;
  digitalWrite(FLASH_PIN, LOW);
}

// ===========================
// Save one frame
// ===========================
void captureFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  char path[64];
  snprintf(path, sizeof(path), "%s/%06lu.jpg", sessionFolder.c_str(), (unsigned long)frameIndex++);

  File f = SD_MMC.open(path, FILE_WRITE);
  if (f) {
    f.write(fb->buf, fb->len);
    f.close();
  }

  esp_camera_fb_return(fb);
}

// ===========================
// Parse received line for REC
// ===========================
void handleLine(const String &line) {
  logLineToSD(line);

  // If telemetry includes ",REC=s" or "REC=s"
  if (line.indexOf("REC=s") >= 0) startRecording();
  if (line.indexOf("REC=e") >= 0) stopRecording();
}

// ===========================
// Setup
// ===========================
void setup() {
  Serial.begin(115200);

  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  bool okCam = initCamera();
  bool okSD  = initSD();

  // minimal status prints
  Serial.println(okCam ? "CAM_OK" : "CAM_FAIL");
  Serial.println(okSD  ? "SD_OK"  : "SD_FAIL");

  // Ensure log file exists
  File f = SD_MMC.open("/car_log.txt", FILE_APPEND);
  if (f) f.close();
}

// ===========================
// Loop
// ===========================
void loop() {
  // Read Serial lines
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      String line = lineBuffer;
      lineBuffer = "";
      line.trim();
      if (line.length() > 0) handleLine(line);
    } else if (c != '\r') {
      lineBuffer += c;
      // avoid huge buffer
      if (lineBuffer.length() > 400) lineBuffer = "";
    }
  }

  // Recording frames at fixed rate
  if (recording) {
    unsigned long now = millis();
    if (now - frameTimer >= FRAME_PERIOD_MS) {
      frameTimer = now;
      captureFrame();
    }
  }
}
////////////////////
/*
  ESP32 CAR Controller
  Core: Arduino-ESP32 3.3.5

  - L298N motor driver (constant speed = 125)
  - IR sensors (LOW = obstacle): 35(L), 39(R), 34(F)
  - Ultrasonic HC-SR04: TRIG=33, ECHO=32
  - MPU6050 I2C: SDA=21, SCL=22
  - RTC DS1302 (Makuna): RST=19, DAT=18, CLK=5
  - Bluetooth commands: F,B,L,R,S
  - Serial2 telemetry to ESP32-CAM (TX=17, RX=16)
*/

#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"

// ===== RTC DS1302 =====
#include <ThreeWire.h>
#include <RtcDS1302.h>

BluetoothSerial SerialBT;

// ===============================
// ====== L298N PINS ======
// ===============================
const int ENA = 25;
const int IN1 = 27;
const int IN2 = 26;

const int ENB = 14;
const int IN3 = 12;
const int IN4 = 13;

// ===============================
// ====== PWM SETTINGS ======
// ===============================
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;      // 8-bit
const int valSpeed = 125;    // duty 0..255

// ===============================
// IR SENSORS (LOW = obstacle)
// ===============================
const int IR_L_PIN = 35;
const int IR_R_PIN = 39;
const int IR_F_PIN = 34;

// ===============================
// Ultrasonic HC-SR04
// ===============================
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;

// ===============================
// RTC DS1302 pins
// ===============================
static const int RTC_RST = 19;
static const int RTC_DAT = 18;
static const int RTC_CLK = 5;

ThreeWire myWire(RTC_DAT, RTC_CLK, RTC_RST);
RtcDS1302<ThreeWire> Rtc(myWire);

// ===============================
// Motion enum
// ===============================
enum Motion { MOT_STOP, MOT_FORWARD, MOT_BACKWARD, MOT_LEFT, MOT_RIGHT };
Motion currentMotion = MOT_STOP;

// ===============================
// Runtime variables
// ===============================
uint8_t irL = 1, irR = 1, irF = 1;
int distanceCM = -1;
float mpuY = 0.0f;
char btCmd = 'S';
char recFlag = 'e';

// ===== timing =====
unsigned long t0 = 0;
bool after3sDone = false;
unsigned long toggleTimer = 0;

unsigned long teleTimer = 0;
const unsigned long TELE_PERIOD_MS = 100;

// ===============================
// PWM helpers (Core 3.x)
// ===============================
inline void pwmWriteA(uint8_t duty) {
  analogWrite(ENA, duty);
}
inline void pwmWriteB(uint8_t duty) {
  analogWrite(ENB, duty);
}

// ===============================
// Motion functions
// ===============================
void stopAll() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  pwmWriteA(0); pwmWriteB(0);
}

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  pwmWriteA(valSpeed); pwmWriteB(valSpeed);
}

void backward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  pwmWriteA(valSpeed); pwmWriteB(valSpeed);
}

void left() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  pwmWriteA(valSpeed); pwmWriteB(valSpeed);
}

void right() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  pwmWriteA(valSpeed); pwmWriteB(valSpeed);
}

void applyState(Motion st) {
  switch (st) {
    case MOT_FORWARD:  forward();  break;
    case MOT_BACKWARD: backward(); break;
    case MOT_LEFT:     left();     break;
    case MOT_RIGHT:    right();    break;
    default:           stopAll();  break;
  }
}

// ===============================
// Ultrasonic
// ===============================
int readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long d = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (d == 0) return -1;
  return (int)(d * 0.0343f / 2.0f);
}

// ===============================
// MPU placeholder
// ===============================
float readMPU_Y_Demo() {
  return 0.0f;
}

// ===============================
// RTC time string
// ===============================
String getRtcTimeString() {
  RtcDateTime now = Rtc.GetDateTime();
  char buf[32];
  snprintf(buf, sizeof(buf),
           "%04u-%02u-%02u %02u:%02u:%02u",
           now.Year(), now.Month(), now.Day(),
           now.Hour(), now.Minute(), now.Second());
  return String(buf);
}

// ===============================
// Update sensors
// ===============================
void updateSensors() {
  irL = digitalRead(IR_L_PIN);
  irR = digitalRead(IR_R_PIN);
  irF = digitalRead(IR_F_PIN);
  distanceCM = readUltrasonicCM();
  mpuY = readMPU_Y_Demo();
}

// ===============================
// recFlag logic
// ===============================
void updateRecFlag() {
  unsigned long now = millis();

  if (!after3sDone) {
    if (now - t0 >= 3000) {
      recFlag = 's';
      after3sDone = true;
      toggleTimer = now;
    } else {
      recFlag = 'e';
    }
    return;
  }

  if (now - toggleTimer >= 15000) {
    recFlag = (recFlag == 's') ? 'e' : 's';
    toggleTimer = now;
  }
}

// ===============================
// Motion logic
// ===============================
void updateMotionFromBT() {
  Motion next = MOT_STOP;

  switch (btCmd) {
    case 'F': next = (irF == 0) ? MOT_STOP : MOT_FORWARD; break;
    case 'B': next = MOT_BACKWARD; break;
    case 'L': next = (irL == 0) ? MOT_STOP : MOT_LEFT; break;
    case 'R': next = (irR == 0) ? MOT_STOP : MOT_RIGHT; break;
    default:  next = MOT_STOP; break;
  }

  currentMotion = next;
  applyState(currentMotion);
}

// ===============================
// Telemetry
// ===============================
void sendTelemetryLine() {
  Serial2.print("TIME=");   Serial2.print(getRtcTimeString());
  Serial2.print(",IR_L=");  Serial2.print(irL);
  Serial2.print(",IR_R=");  Serial2.print(irR);
  Serial2.print(",IR_F=");  Serial2.print(irF);
  Serial2.print(",DIST=");  Serial2.print(distanceCM);
  Serial2.print(",MPU_Y="); Serial2.print(mpuY, 3);
  Serial2.print(",BT=");    Serial2.print(btCmd);
  Serial2.print(",MOTION=");Serial2.print((int)currentMotion);
  Serial2.print(",REC=");   Serial2.println(recFlag);
}

// ===============================
// Setup
// ===============================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // ✅ Core 3.x correct PWM init
  analogWriteResolution(ENA, PWM_RES);
  analogWriteResolution(ENB, PWM_RES);
  analogWriteFrequency(ENA, PWM_FREQ);
  analogWriteFrequency(ENB, PWM_FREQ);

  stopAll();

  pinMode(IR_L_PIN, INPUT);
  pinMode(IR_R_PIN, INPUT);
  pinMode(IR_F_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();

  Rtc.Begin();
  if (!Rtc.GetIsRunning()) Rtc.SetIsRunning(true);

  SerialBT.begin("ESP32_CAR");
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  t0 = millis();
  teleTimer = millis();
}

// ===============================
// Loop
// ===============================
void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c=='F'||c=='B'||c=='L'||c=='R'||c=='S') btCmd = c;
  }

  updateSensors();
  updateRecFlag();
  updateMotionFromBT();

  unsigned long now = millis();
  if (now - teleTimer >= TELE_PERIOD_MS) {
    teleTimer = now;
    sendTelemetryLine();
  }
}
