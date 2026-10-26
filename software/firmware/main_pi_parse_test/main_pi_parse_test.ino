#include <Arduino.h>

#include "../common/bn_ota.h"
#include "../common/bn_can_protocol.h"

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t PI_COMMAND_TIMEOUT_MS = 500;
static const uint32_t STATUS_PERIOD_MS = 500;
static const size_t RX_LINE_MAX = 256;

static String rxLine;
static uint32_t lastPiCommandMs = 0;
static uint32_t lastStatusMs = 0;
static long lastSeq = -1;

static float lastThrottle = 0.0f;
static float lastSteering = 0.0f;
static uint16_t leftPwmUs = bncan::PWM_NEUTRAL_US;
static uint16_t rightPwmUs = bncan::PWM_NEUTRAL_US;

static uint16_t clampPwmInt(int pwm) {
  if (pwm < bncan::PWM_MIN_US) return bncan::PWM_MIN_US;
  if (pwm > bncan::PWM_MAX_US) return bncan::PWM_MAX_US;
  return (uint16_t)pwm;
}

static float clampNorm(float value) {
  if (value < -1.0f) return -1.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

static uint16_t normalizedToPwm(float value) {
  value = clampNorm(value);
  return clampPwmInt((int)(bncan::PWM_NEUTRAL_US + value * 500.0f));
}

static bool extractJsonNumber(const String &line, const char *key, float &outValue) {
  String quotedKey = String("\"") + key + "\"";
  int keyIndex = line.indexOf(quotedKey);
  if (keyIndex < 0) return false;

  int colonIndex = line.indexOf(':', keyIndex + quotedKey.length());
  if (colonIndex < 0) return false;

  int start = colonIndex + 1;
  while (start < (int)line.length() && isspace((unsigned char)line[start])) {
    start++;
  }

  int end = start;
  while (end < (int)line.length()) {
    char c = line[end];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
      end++;
    } else {
      break;
    }
  }

  if (end == start) return false;
  outValue = line.substring(start, end).toFloat();
  return true;
}

static bool parseManualCommand(const String &line, long &seqOut, float &throttleOut, float &steeringOut) {
  if (line.indexOf("\"type\"") < 0 || line.indexOf("cmd_manual") < 0) {
    return false;
  }

  float seqValue = -1.0f;
  extractJsonNumber(line, "seq", seqValue);
  seqOut = (long)seqValue;

  if (!extractJsonNumber(line, "throttle", throttleOut)) return false;
  if (!extractJsonNumber(line, "steering", steeringOut)) return false;

  if (throttleOut < -1.0f || throttleOut > 1.0f) return false;
  if (steeringOut < -1.0f || steeringOut > 1.0f) return false;
  return true;
}

static void mixToLeftRight(float throttle, float steering, uint16_t &leftOut, uint16_t &rightOut) {
  int thr = (int)normalizedToPwm(throttle) - bncan::PWM_NEUTRAL_US;
  int str = (int)normalizedToPwm(steering) - bncan::PWM_NEUTRAL_US;

  leftOut = clampPwmInt((int)bncan::PWM_NEUTRAL_US + thr + str);
  rightOut = clampPwmInt((int)bncan::PWM_NEUTRAL_US + thr - str);
}

static bool piFresh() {
  return lastPiCommandMs != 0 && (millis() - lastPiCommandMs <= PI_COMMAND_TIMEOUT_MS);
}

static void forceNeutral() {
  leftPwmUs = bncan::PWM_NEUTRAL_US;
  rightPwmUs = bncan::PWM_NEUTRAL_US;
}

static void sendAck(long seq, float throttle, float steering, uint16_t leftPwm, uint16_t rightPwm) {
  Serial.print("{\"type\":\"ack\",\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"seq\":");
  Serial.print(seq);
  Serial.print(",\"data\":{\"accepted\":true,\"throttle\":");
  Serial.print(throttle, 3);
  Serial.print(",\"steering\":");
  Serial.print(steering, 3);
  Serial.print(",\"left_pwm\":");
  Serial.print(leftPwm);
  Serial.print(",\"right_pwm\":");
  Serial.print(rightPwm);
  Serial.println("}}");
}

static void sendReject(const char *reason) {
  Serial.print("{\"type\":\"ack\",\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"seq\":-1,\"data\":{\"accepted\":false,\"reason\":\"");
  Serial.print(reason);
  Serial.println("\"}}");
}

static void sendStatus(const char *event = nullptr) {
  Serial.print("{\"type\":\"main_parse_test_status\",\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"seq\":");
  Serial.print(lastSeq);
  Serial.print(",\"data\":{\"pi_fresh\":");
  Serial.print(piFresh() ? "true" : "false");
  Serial.print(",\"throttle\":");
  Serial.print(lastThrottle, 3);
  Serial.print(",\"steering\":");
  Serial.print(lastSteering, 3);
  Serial.print(",\"left_pwm\":");
  Serial.print(leftPwmUs);
  Serial.print(",\"right_pwm\":");
  Serial.print(rightPwmUs);
  Serial.print(",\"pi_timeout_ms\":");
  Serial.print(PI_COMMAND_TIMEOUT_MS);
  if (event != nullptr) {
    Serial.print(",\"event\":\"");
    Serial.print(event);
    Serial.print("\"");
  }
  Serial.println("}}");
}

static void handleLine(const String &line) {
  long seq = -1;
  float throttle = 0.0f;
  float steering = 0.0f;

  if (!parseManualCommand(line, seq, throttle, steering)) {
    sendReject("PARSE_ERROR");
    return;
  }

  lastSeq = seq;
  lastThrottle = throttle;
  lastSteering = steering;
  lastPiCommandMs = millis();

  mixToLeftRight(lastThrottle, lastSteering, leftPwmUs, rightPwmUs);
  sendAck(lastSeq, lastThrottle, lastSteering, leftPwmUs, rightPwmUs);
}

static void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxLine.length() > 0) {
        handleLine(rxLine);
      }
      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() < RX_LINE_MAX) {
        rxLine += c;
      } else {
        rxLine = "";
        sendReject("LINE_TOO_LONG");
      }
    }
  }
}

void setup() {
  pinMode(RGB_BUILTIN, OUTPUT);
  digitalWrite(RGB_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(200);

  forceNeutral();
  bnOtaBegin("bn-usv-main-parse-test");

  Serial.println("{\"type\":\"main_parse_test_ready\",\"data\":{\"baud\":115200}}");
  sendStatus("boot_neutral");
}

void loop() {
  bnOtaHandle();
  readSerialLines();

  if (!piFresh()) {
    forceNeutral();
  }

  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus();
  }

  delay(5);
}

