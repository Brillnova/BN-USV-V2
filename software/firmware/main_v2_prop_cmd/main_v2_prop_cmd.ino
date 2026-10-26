#include <Arduino.h>
#include <driver/twai.h>

// Uncomment to suppress development-only USB Serial diagnostics. GCS telemetry
// remains enabled through BN_TELEMETRY_SERIAL.
// #define BN_DEBUG_SERIAL 0

#include "../common/bn_ota.h"
#include "../common/bn_can_protocol.h"
#include "../common/bn_debug.h"
#include "../common/bn_can_transport.h"
#include "../common/bn_main_control.h"
#include "../common/bn_battery_monitor.h"

static const int RC_PINS[3] = {4, 5, 6};  // Steering, throttle, mode.
static portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t rcRiseUs[3] = {};
static volatile bool rcRisingSeen[3] = {};
static volatile uint16_t rcWidths[3] = {1500, 1500, 1500};
static volatile uint32_t rcLastMs[3] = {};
static volatile bool rcSeen[3] = {};
static bncontrol::Controller controller;
static bncontrol::Input controlInput;
static bncontrol::Output controlOutput;
static bool otaLocked = false;

static void IRAM_ATTR captureRc(unsigned index) {
  const bool high = digitalRead(RC_PINS[index]);
  const uint32_t nowUs = micros();
  portENTER_CRITICAL_ISR(&rcMux);
  if (high) {
    rcRiseUs[index] = nowUs;
    rcRisingSeen[index] = true;
  } else if (rcRisingSeen[index]) {
    const uint32_t width = nowUs - rcRiseUs[index];
    rcRisingSeen[index] = false;
    if (width >= 900 && width <= 2100) {
      rcWidths[index] = uint16_t(width);
      rcLastMs[index] = millis();
      rcSeen[index] = true;
    }
  }
  portEXIT_CRITICAL_ISR(&rcMux);
}
void IRAM_ATTR rcSteeringIsr() { captureRc(0); }
void IRAM_ATTR rcThrottleIsr() { captureRc(1); }
void IRAM_ATTR rcModeIsr() { captureRc(2); }

static uint32_t snapshotRc() {
  portENTER_CRITICAL(&rcMux);
  controlInput.steering = {rcWidths[0], rcLastMs[0], rcSeen[0]};
  controlInput.throttle = {rcWidths[1], rcLastMs[1], rcSeen[1]};
  controlInput.mode = {rcWidths[2], rcLastMs[2], rcSeen[2]};
  const uint32_t now = millis();
  portEXIT_CRITICAL(&rcMux);
  return now;
}

// Main ESP32-S3 Rev.A CAN pins.
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_7;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_8;
static bntransport::Bus canBus(CAN_TX_PIN, CAN_RX_PIN);
static uint32_t canEpoch = 0;

// Main Rev.A: XT60 auxiliary sense lead -> 100k / 15k divider -> GPIO9.
static const int MAIN_VBAT_ADC_PIN = 9;
static const uint32_t MAIN_BATTERY_SAMPLE_PERIOD_MS = 10;
static bnbattery::Monitor mainBattery;
static uint32_t lastMainBatterySampleMs = 0;

static const uint32_t SERIAL_BAUD = 115200;
static const uint16_t PWM_NEU_US = 1500;
static const uint16_t PWM_MIN_US = 1000;
static const uint16_t PWM_MAX_US = 2000;
static const uint8_t PROP_TIMEOUT_100MS = 5;
static const uint32_t ARM_CMD_PERIOD_MS = 200;

static uint16_t propSeq = 0;
static uint16_t armSeq = 0;
static uint32_t lastPropTxMs = 0;
static uint32_t lastArmTxMs = 0;
static uint32_t lastMainStatusMs = 0;
static uint32_t lastLeftStatusMs = 0;
static uint32_t lastRightStatusMs = 0;
static bool canReady = false;
static bnrecovery::MainGuard canGuard;
static bool leftSeen = false, rightSeen = false;
static bool leftMainFresh = false, rightMainFresh = false;
// Both propulsion nodes are required. This does not prevent CAN bench diagnostics.
static bool peersFresh(uint32_t now) {
  return leftSeen && rightSeen && leftMainFresh && rightMainFresh &&
      uint32_t(now - lastLeftStatusMs) <= 1000 && uint32_t(now - lastRightStatusMs) <= 1000;
}

static uint16_t clampPwmInt(int pwm) {
  if (pwm < PWM_MIN_US) return PWM_MIN_US;
  if (pwm > PWM_MAX_US) return PWM_MAX_US;
  return (uint16_t)pwm;
}

static uint16_t normalizedToPwm(float value) {
  if (value < -1.0f) value = -1.0f;
  if (value > 1.0f) value = 1.0f;
  return clampPwmInt((int)(PWM_NEU_US + value * 500.0f));
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

static bool parseManualCommand(const String &line, float &throttle, float &steering) {
  if (line.indexOf("\"type\"") < 0 || line.indexOf("cmd_manual") < 0) {
    return false;
  }
  if (!extractJsonNumber(line, "throttle", throttle)) return false;
  if (!extractJsonNumber(line, "steering", steering)) return false;
  if (throttle < -1.0f || throttle > 1.0f) return false;
  if (steering < -1.0f || steering > 1.0f) return false;
  return true;
}

static bool canSendFrame(const bncan::CanFrame &frame) {
  return canBus.send(frame);
}

static bool canReceiveFrame(bncan::CanFrame &frame) {
  return canBus.receive(frame);
}

static void sendPropCmd(uint8_t targetNode, uint16_t pwmUs, uint8_t flags) {
  bncan::PropCmd cmd = {};
  cmd.seq = propSeq++;
  cmd.pwm_us = pwmUs;
  cmd.flags = flags;
  cmd.timeout_100ms = PROP_TIMEOUT_100MS;

  bncan::CanFrame frame = {};
  bncan::packPropCmd(targetNode, cmd, frame);
  canSendFrame(frame);
}

static void sendArmCmd(uint8_t targetNode, uint8_t armState, uint8_t reasonCode) {
  bncan::ArmCmd cmd = {};
  cmd.seq = armSeq++;
  cmd.arm_state = armState;
  cmd.reason_code = reasonCode;

  bncan::CanFrame frame = {};
  bncan::packArmCmd(targetNode, cmd, frame);
  canSendFrame(frame);
}

static void handleSubStatus(const bncan::CanFrame &frame) {
  bncan::SubStatus status = {};
  uint8_t sourceNode = 0;

  if (bncan::unpackSubStatus(frame, bncan::NODE_LEFT_SUB, status)) {
    sourceNode = bncan::NODE_LEFT_SUB;
    lastLeftStatusMs = millis();
    leftSeen = true;
    leftMainFresh = (status.status_flags & bncan::STATUS_FLAG_MAIN_FRESH) != 0;
  } else if (bncan::unpackSubStatus(frame, bncan::NODE_RIGHT_SUB, status)) {
    sourceNode = bncan::NODE_RIGHT_SUB;
    lastRightStatusMs = millis();
    rightSeen = true;
    rightMainFresh = (status.status_flags & bncan::STATUS_FLAG_MAIN_FRESH) != 0;
  } else {
    return;
  }

  if (BN_TELEMETRY_SERIAL) {
    Serial.print("{\"type\":\"sub_status\",\"timestamp\":");
    Serial.print(millis() / 1000.0f, 3);
    Serial.print(",\"data\":{\"source_node\":");
    Serial.print(sourceNode);
    Serial.print(",\"seq\":");
    Serial.print(status.seq);
    Serial.print(",\"actual_pwm_us\":");
    Serial.print(status.actual_pwm_us);
    Serial.print(",\"vbat_v\":");
    Serial.print(status.vbat_mv_div10 / 100.0f, 2);
    Serial.print(",\"status_flags\":");
    Serial.print(status.status_flags);
    Serial.print(",\"fault_flags\":");
    Serial.print(status.fault_flags);
    Serial.println("}}");
  }
}

static esp_err_t canBegin() { return canBus.begin(); }

static void sendMainStatus(bool commandFresh, uint16_t leftPwmUs, uint16_t rightPwmUs) {
  uint32_t nowMs = millis();

  if (BN_TELEMETRY_SERIAL) {
    Serial.print("{\"type\":\"main_status\",\"timestamp\":");
    Serial.print(nowMs / 1000.0f, 3);
    Serial.print(",\"data\":{\"can_ok\":");
    Serial.print(canReady ? "true" : "false");
    // CAN health is operational telemetry used by the Pi test logger. Keep it
    // available even when development-only USB diagnostic messages are off.
    canBus.printDiagnostics();
    Serial.print(",\"can_rearm_required\":");
    Serial.print(canGuard.locked() ? "true" : "false");
    Serial.print(",\"can_peers_fresh\":");
    Serial.print(peersFresh(nowMs) ? "true" : "false");
    Serial.print(",\"pi_command_fresh\":");
    Serial.print(commandFresh ? "true" : "false");
    Serial.print(",\"mode\":\"");
    Serial.print(bncontrol::name(controlOutput.mode));
    Serial.print("\",\"control_source\":\"");
    Serial.print(controlOutput.enabled() ? (controlOutput.mode == bncontrol::Mode::Manual ? "RC" : "PI") : bncontrol::name(controlOutput.mode));
    Serial.print("\",\"rc_mode\":\"");
    Serial.print(bncontrol::name(controlOutput.rcMode));
    Serial.print("\",\"rc_ok\":");
    Serial.print(controlOutput.rcOk ? "true" : "false");
    Serial.print(",\"manual_latch\":");
    Serial.print(controlOutput.manualLatch ? "true" : "false");
    Serial.print(",\"ch1_us\":");
    Serial.print(controlInput.steering.us);
    Serial.print(",\"ch2_us\":");
    Serial.print(controlInput.throttle.us);
    Serial.print(",\"ch4_us\":");
    Serial.print(controlInput.mode.us);
    Serial.print(",\"cmd_thr_us\":");
    Serial.print(controlOutput.throttle);
    Serial.print(",\"cmd_str_us\":");
    Serial.print(controlOutput.steering);
    Serial.print(",\"ota_locked\":");
    Serial.print(otaLocked ? "true" : "false");
    const bool batteryValid = mainBattery.valid(nowMs);
    Serial.print(",\"vbat_v\":");
    if (batteryValid) Serial.print(mainBattery.voltageDiv10(nowMs) / 100.0f, 2);
    else Serial.print("null");
    Serial.print(",\"battery_valid\":");
    Serial.print(batteryValid ? "true" : "false");
    Serial.print(",\"left_pwm_us\":");
    Serial.print(leftPwmUs);
    Serial.print(",\"right_pwm_us\":");
    Serial.print(rightPwmUs);
    Serial.print(",\"left_status_age_ms\":");
    Serial.print(lastLeftStatusMs == 0 ? -1 : (int32_t)(nowMs - lastLeftStatusMs));
    Serial.print(",\"right_status_age_ms\":");
    Serial.print(lastRightStatusMs == 0 ? -1 : (int32_t)(nowMs - lastRightStatusMs));
    Serial.println("}}");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  BN_DEBUG_PRINTLN("{\"type\":\"main_v2_boot\",\"data\":{\"stage\":\"serial_ready\"}}");

  esp_err_t canErr = canBegin();
  canReady = canErr == ESP_OK;
  if (BN_DEBUG_SERIAL) {
    Serial.print("{\"type\":\"main_v2_ready\",\"data\":{\"can_ok\":");
    Serial.print(canReady ? "true" : "false");
    Serial.print(",\"can_err\":");
    Serial.print((int)canErr);
    Serial.println(",\"can_bitrate\":500000}}");
  }

  bnOtaBegin("bn-usv-main");
  for (int pin : RC_PINS) pinMode(pin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[0]), rcSteeringIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[1]), rcThrottleIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[2]), rcModeIsr, CHANGE);
  pinMode(MAIN_VBAT_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MAIN_VBAT_ADC_PIN, ADC_11db);
#if ENABLE_OTA
  ArduinoOTA.onStart([]() {
    otaLocked = true;
    sendArmCmd(bncan::NODE_LEFT_SUB, bncan::ARM_STATE_KILL, bncan::ARM_REASON_OPERATOR_DISARM);
    sendArmCmd(bncan::NODE_RIGHT_SUB, bncan::ARM_STATE_KILL, bncan::ARM_REASON_OPERATOR_DISARM);
    sendPropCmd(bncan::NODE_LEFT_SUB, 1500, bncan::PROP_FLAG_FAILSAFE_ACTIVE);
    sendPropCmd(bncan::NODE_RIGHT_SUB, 1500, bncan::PROP_FLAG_FAILSAFE_ACTIVE);
  });
  // A failed OTA attempt remains locked until Main is rebooted.
#endif
}

void loop() {
  bnOtaHandle();
  const uint32_t loopNowMs = millis();
  if (loopNowMs - lastMainBatterySampleMs >= MAIN_BATTERY_SAMPLE_PERIOD_MS) {
    lastMainBatterySampleMs = loopNowMs;
    mainBattery.add(analogReadMilliVolts(MAIN_VBAT_ADC_PIN), loopNowMs);
  }
  canBus.poll(millis());
  canReady = canBus.running();
  if (canEpoch != canBus.epoch || !canReady) {
    canEpoch = canBus.epoch;
    canGuard.trip();
    leftSeen = rightSeen = false;
    controlInput.piSeen = false;
    controller = bncontrol::Controller();
  }

  static String rxLine;
  static bool discardLine = false;

  for (unsigned count = 0; count < 256 && Serial.available() > 0; ++count) {
    char c = (char)Serial.read();
    if (c == '\n') {
      float throttle = 0.0f;
      float steering = 0.0f;
      if (!discardLine && !canGuard.locked() && parseManualCommand(rxLine, throttle, steering)) {
        controlInput.piThrottle = normalizedToPwm(throttle);
        controlInput.piSteering = normalizedToPwm(steering);
        controlInput.piLastMs = millis();
        controlInput.piSeen = true;
        if (BN_DEBUG_SERIAL) {
          Serial.print("{\"type\":\"pi_cmd_rx\",\"data\":{\"throttle\":");
          Serial.print(throttle, 3);
          Serial.print(",\"steering\":");
          Serial.print(steering, 3);
          Serial.println("}}");
        }
      }
      rxLine = "";
      discardLine = false;
    } else if (c != '\r') {
      if (!discardLine && rxLine.length() < 256) rxLine += c;
      else { discardLine = true; rxLine = ""; }
    }
  }

  uint32_t nowMs = millis();

  bncan::CanFrame rxFrame = {};
  for (unsigned count = 0; count < 16 && canReceiveFrame(rxFrame); ++count) {
    handleSubStatus(rxFrame);
  }

  if (nowMs - lastPropTxMs >= 50) {
    nowMs = snapshotRc();
    lastPropTxMs = nowMs;
    const bncontrol::Mode previousMode = controlOutput.mode;
    const bool wasLocked = canGuard.locked();
    const bool rcReset = bncontrol::fresh(controlInput.steering, nowMs, 300) &&
        bncontrol::fresh(controlInput.throttle, nowMs, 300) && bncontrol::fresh(controlInput.mode, nowMs, 500) &&
        controlInput.mode.us >= 1700 &&
        bncontrol::magnitude(int(controlInput.steering.us) - 1500) < 60 &&
        bncontrol::magnitude(int(controlInput.throttle.us) - 1500) < 60;
    canGuard.update(canReady, peersFresh(nowMs), rcReset);
    if (wasLocked != canGuard.locked()) {
      canBus.discardPending();
      controller = bncontrol::Controller();
      controlInput.piSeen = false;
    }
    controlInput.canReady = canReady && !otaLocked && !canGuard.locked();
    controlOutput = controller.update(controlInput, nowMs);
    if (nowMs - lastArmTxMs >= ARM_CMD_PERIOD_MS || previousMode != controlOutput.mode) {
      lastArmTxMs = nowMs;
      uint8_t state = controlOutput.enabled() ? bncan::ARM_STATE_ARM : bncan::ARM_STATE_DISARM;
      uint8_t reason = bncan::ARM_REASON_NORMAL;
      if (controlOutput.mode == bncontrol::Mode::Kill || otaLocked) {
        state = bncan::ARM_STATE_KILL;
        reason = bncan::ARM_REASON_OPERATOR_DISARM;
      } else if (controlOutput.reason == bncontrol::Reason::RcLost) reason = bncan::ARM_REASON_RC_LOST;
      else if (controlOutput.reason == bncontrol::Reason::PiLost) reason = bncan::ARM_REASON_PI_LOST;
      else if (controlOutput.reason == bncontrol::Reason::CanUnavailable) reason = bncan::ARM_REASON_SUB_FAULT;
      sendArmCmd(bncan::NODE_LEFT_SUB, state, reason);
      sendArmCmd(bncan::NODE_RIGHT_SUB, state, reason);
    }
    uint8_t flags = bncan::PROP_FLAG_FAILSAFE_ACTIVE;
    if (controlOutput.enabled()) flags = bncan::PROP_FLAG_ARM_ALLOWED |
        (controlOutput.mode == bncontrol::Mode::Manual ? bncan::PROP_FLAG_RC_AUTHORITY : bncan::PROP_FLAG_AUTO_AUTHORITY);
    sendPropCmd(bncan::NODE_LEFT_SUB, controlOutput.left, flags);
    sendPropCmd(bncan::NODE_RIGHT_SUB, controlOutput.right, flags);
  }

  if (nowMs - lastMainStatusMs >= 1000) {
    lastMainStatusMs = nowMs;
    sendMainStatus(controlOutput.piFresh, controlOutput.left, controlOutput.right);
  }
}
