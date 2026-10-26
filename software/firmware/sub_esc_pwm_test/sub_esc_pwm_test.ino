#include <Arduino.h>
#include <ESP32Servo.h>

#include "../common/bn_ota.h"
#include "../common/bn_can_protocol.h"

// Tenstar Robot ESP32-C3 Super Mini Rev.A ESC output pin.
static const int ESC_PWM_PIN = 10;

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t COMMAND_TIMEOUT_MS = 3000;
static const uint32_t STATUS_PERIOD_MS = 500;

Servo esc;

static String rxLine;
static bool armed = false;
static uint16_t targetPwmUs = bncan::PWM_NEUTRAL_US;
static uint16_t activePwmUs = bncan::PWM_NEUTRAL_US;
static uint32_t lastCommandMs = 0;
static uint32_t lastStatusMs = 0;

static uint16_t clampPwmInt(int pwm) {
  if (pwm < bncan::PWM_MIN_US) return bncan::PWM_MIN_US;
  if (pwm > bncan::PWM_MAX_US) return bncan::PWM_MAX_US;
  return (uint16_t)pwm;
}

static void writeEsc(uint16_t pwmUs) {
  activePwmUs = clampPwmInt(pwmUs);
  esc.writeMicroseconds(activePwmUs);
}

static void forceNeutral() {
  targetPwmUs = bncan::PWM_NEUTRAL_US;
  writeEsc(targetPwmUs);
}

static void sendStatus(const char *event = nullptr) {
  Serial.print("{\"type\":\"esc_test_status\",\"timestamp\":");
  Serial.print(millis() / 1000.0f, 3);
  Serial.print(",\"data\":{");
  Serial.print("\"armed\":");
  Serial.print(armed ? "true" : "false");
  Serial.print(",\"target_pwm_us\":");
  Serial.print(targetPwmUs);
  Serial.print(",\"active_pwm_us\":");
  Serial.print(activePwmUs);
  Serial.print(",\"timeout_ms\":");
  Serial.print(COMMAND_TIMEOUT_MS);
  if (event != nullptr) {
    Serial.print(",\"event\":\"");
    Serial.print(event);
    Serial.print("\"");
  }
  Serial.println("}}");
}

static bool parsePwmCommand(const String &line, uint16_t &pwmOut) {
  if (line.length() == 0) return false;

  int start = 0;
  while (start < (int)line.length() && isspace((unsigned char)line[start])) {
    start++;
  }

  int end = start;
  while (end < (int)line.length() && isdigit((unsigned char)line[end])) {
    end++;
  }

  if (end == start) return false;

  int pwm = line.substring(start, end).toInt();
  if (pwm < bncan::PWM_MIN_US || pwm > bncan::PWM_MAX_US) return false;

  pwmOut = (uint16_t)pwm;
  return true;
}

static void handleCommand(String line) {
  line.trim();
  line.toLowerCase();

  if (line == "arm") {
    armed = true;
    lastCommandMs = millis();
    forceNeutral();
    sendStatus("armed_neutral");
    return;
  }

  if (line == "disarm" || line == "kill" || line == "stop") {
    armed = false;
    forceNeutral();
    sendStatus("disarmed_neutral");
    return;
  }

  if (line == "neutral" || line == "n") {
    lastCommandMs = millis();
    forceNeutral();
    sendStatus("neutral");
    return;
  }

  if (line == "status" || line == "?") {
    sendStatus("status_requested");
    return;
  }

  uint16_t pwm = bncan::PWM_NEUTRAL_US;
  if (parsePwmCommand(line, pwm)) {
    lastCommandMs = millis();
    targetPwmUs = pwm;
    if (armed) {
      writeEsc(targetPwmUs);
      sendStatus("pwm_applied");
    } else {
      forceNeutral();
      sendStatus("pwm_rejected_disarmed");
    }
    return;
  }

  Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_COMMAND\"}}");
}

static void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() < 80) {
        rxLine += c;
      } else {
        rxLine = "";
        Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"LINE_TOO_LONG\"}}");
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  esc.setPeriodHertz(50);
  esc.attach(ESC_PWM_PIN, bncan::PWM_MIN_US, bncan::PWM_MAX_US);
  forceNeutral();

  bnOtaBegin("bn-usv-sub-esc-test");

  Serial.println("{\"type\":\"esc_test_ready\",\"data\":{\"pin\":10,\"neutral_pwm_us\":1500}}");
  sendStatus("boot_neutral");
}

void loop() {
  bnOtaHandle();
  readSerialCommands();

  uint32_t nowMs = millis();
  bool commandTimedOut = armed && lastCommandMs != 0 && (nowMs - lastCommandMs > COMMAND_TIMEOUT_MS);
  if (commandTimedOut) {
    armed = false;
    forceNeutral();
    sendStatus("command_timeout_disarmed");
  }

  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus();
  }

  delay(5);
}

