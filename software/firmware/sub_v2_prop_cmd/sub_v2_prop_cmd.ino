#include <Arduino.h>
#include <ESP32Servo.h>
#include <driver/twai.h>

// Uncomment for production builds that should not print USB Serial diagnostics.
// #define BN_DEBUG_SERIAL 0

#include "../common/bn_ota.h"
#include "../common/bn_can_protocol.h"
#include "../common/bn_debug.h"
#include "../common/bn_can_transport.h"
#include "../common/bn_battery_monitor.h"

// Build one firmware image for each Sub at first.
#ifndef SUB_NODE_ID
#define SUB_NODE_ID bncan::NODE_RIGHT_SUB
#endif

// Tenstar Robot ESP32-C3 Super Mini Rev.A pins.
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_1;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_3;
static bntransport::Bus canBus(CAN_TX_PIN, CAN_RX_PIN);
static uint32_t canEpoch = 0;
static const int ESC_PWM_PIN = 10;
static const int VBAT_ADC_PIN = 0;
static const uint32_t BATTERY_SAMPLE_PERIOD_MS = 10;

static const uint32_t PROP_TIMEOUT_DEFAULT_MS = 500;
static const uint32_t ESC_ARMING_WAIT_MS = 5000;
static const uint32_t STATUS_PERIOD_MS = 200;
static bnbattery::Monitor battery;
static uint32_t lastBatterySampleMs = 0;

Servo esc;

static uint16_t activePwmUs = bncan::PWM_NEUTRAL_US;
static uint32_t lastValidPropMs = 0;
static uint32_t activeTimeoutMs = PROP_TIMEOUT_DEFAULT_MS;
static uint32_t bootMs = 0;
static uint32_t lastStatusTxMs = 0;
static uint16_t statusSeq = 0;
static bool canOk = false;
static bool armed = false;
static uint8_t lastArmReason = bncan::ARM_REASON_STARTUP;
static uint8_t faultFlags = bncan::FAULT_FLAG_COMMAND_TIMEOUT;

static bool canBegin() { return canBus.begin() == ESP_OK; }

static bool canReceiveFrame(bncan::CanFrame &frame) {
  return canBus.receive(frame);
}

static bool canSendFrame(const bncan::CanFrame &frame) {
  return canBus.send(frame);
}

static void writeEscNeutral() {
  activePwmUs = bncan::PWM_NEUTRAL_US;
  esc.writeMicroseconds(activePwmUs);
}

static bool escArmingWaitDone() {
  return millis() - bootMs >= ESC_ARMING_WAIT_MS;
}

static bool propCommandFresh() {
  return lastValidPropMs != 0 && (millis() - lastValidPropMs <= activeTimeoutMs);
}

static void sendStatus(const char *event = nullptr) {
  bncan::SubStatus status = {};
  status.seq = statusSeq++;
  status.actual_pwm_us = activePwmUs;
  const uint32_t statusMs = millis();
  const bool batteryValid = battery.valid(statusMs);
  status.vbat_mv_div10 = battery.voltageDiv10(statusMs);
  status.status_flags = 0;
  status.fault_flags = faultFlags;
  // Measurement diagnostics only; low-voltage propulsion policy is not yet implemented.
  if (!batteryValid) status.fault_flags |= bncan::FAULT_FLAG_ADC_FAULT;
  // Leave VBAT_OK unset until a battery operating-voltage policy is defined.

  if (armed) status.status_flags |= bncan::STATUS_FLAG_ARMED;
  if (propCommandFresh()) status.status_flags |= bncan::STATUS_FLAG_MAIN_FRESH;
  if (armed && escArmingWaitDone() && propCommandFresh() && faultFlags == 0) {
    status.status_flags |= bncan::STATUS_FLAG_ESC_OUTPUT_ENABLED;
  }
  status.status_flags |= bncan::STATUS_FLAG_LOCAL_5V_OK;
  if (canOk) status.status_flags |= bncan::STATUS_FLAG_CAN_OK;

  bncan::CanFrame frame = {};
  bncan::packSubStatus((uint8_t)SUB_NODE_ID, status, frame);
  bool txOk = canSendFrame(frame);

  if (BN_DEBUG_SERIAL) {
    Serial.print("{\"type\":\"sub_status\",\"data\":{\"node_id\":");
    Serial.print((uint8_t)SUB_NODE_ID);
    Serial.print(",\"event\":\"");
    Serial.print(event == nullptr ? "periodic" : event);
    Serial.print("\",\"armed\":");
    Serial.print(armed ? "true" : "false");
    Serial.print(",\"arming_wait_done\":");
    Serial.print(escArmingWaitDone() ? "true" : "false");
    Serial.print(",\"main_fresh\":");
    Serial.print(propCommandFresh() ? "true" : "false");
    Serial.print(",\"active_pwm_us\":");
    Serial.print(activePwmUs);
    Serial.print(",\"vbat_v\":");
    if (batteryValid) Serial.print(status.vbat_mv_div10 / 100.0f, 2);
    else Serial.print("null");
    Serial.print(",\"battery_valid\":");
    Serial.print(batteryValid ? "true" : "false");
    Serial.print(",\"fault_flags\":");
    Serial.print(status.fault_flags);
    Serial.print(",\"last_arm_reason\":");
    Serial.print(lastArmReason);
    Serial.print(",\"can_tx_ok\":");
    Serial.print(txOk ? "true" : "false");
    canBus.printDiagnostics();
    Serial.println("}}");
  }
}

static void handleArmCmd(const bncan::CanFrame &frame) {
  bncan::ArmCmd cmd = {};
  if (!bncan::unpackArmCmd(frame, (uint8_t)SUB_NODE_ID, cmd)) return;

  lastArmReason = cmd.reason_code;

  if (cmd.arm_state == bncan::ARM_STATE_ARM) {
    armed = true;
    faultFlags &= ~bncan::FAULT_FLAG_CRITICAL_FAULT;
    BN_DEBUG_PRINTLN("{\"type\":\"sub_arm\",\"data\":{\"state\":\"ARM\"}}");
    return;
  }

  armed = false;
  writeEscNeutral();

  if (cmd.arm_state == bncan::ARM_STATE_KILL) {
    faultFlags |= bncan::FAULT_FLAG_CRITICAL_FAULT;
    BN_DEBUG_PRINTLN("{\"type\":\"sub_arm\",\"data\":{\"state\":\"KILL\",\"output\":\"neutral\"}}");
  } else {
    BN_DEBUG_PRINTLN("{\"type\":\"sub_arm\",\"data\":{\"state\":\"DISARM\",\"output\":\"neutral\"}}");
  }
}

static void handlePropCmd(const bncan::CanFrame &frame) {
  bncan::PropCmd cmd = {};
  if (!bncan::unpackPropCmd(frame, (uint8_t)SUB_NODE_ID, cmd)) return;

  lastValidPropMs = millis();
  activeTimeoutMs = cmd.timeout_100ms > 0 ? (uint32_t)cmd.timeout_100ms * 100u : PROP_TIMEOUT_DEFAULT_MS;
  faultFlags &= ~bncan::FAULT_FLAG_COMMAND_TIMEOUT;

  bool commandAllowsOutput = bncan::propCmdAllowsOutput(cmd);
  bool outputAllowed = canOk && armed && escArmingWaitDone() && commandAllowsOutput;

  if (outputAllowed) {
    activePwmUs = cmd.pwm_us;
    esc.writeMicroseconds(activePwmUs);
  } else {
    writeEscNeutral();
  }

  if (BN_DEBUG_SERIAL) {
    Serial.print("{\"type\":\"prop_cmd_rx\",\"data\":{\"node_id\":");
    Serial.print((uint8_t)SUB_NODE_ID);
    Serial.print(",\"seq\":");
    Serial.print(cmd.seq);
    Serial.print(",\"pwm_us\":");
    Serial.print(cmd.pwm_us);
    Serial.print(",\"output_allowed\":");
    Serial.print(outputAllowed ? "true" : "false");
    Serial.println("}}");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  bootMs = millis();

  esc.setPeriodHertz(50);
  esc.attach(ESC_PWM_PIN, bncan::PWM_MIN_US, bncan::PWM_MAX_US);
  writeEscNeutral();

  pinMode(VBAT_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);

  canOk = canBegin();
  if (!canOk) {
    BN_DEBUG_PRINTLN("CAN_INIT_FAILED");
  }

  const char *otaHostname =
      ((uint8_t)SUB_NODE_ID == bncan::NODE_RIGHT_SUB) ? "bn-usv-sub-right" : "bn-usv-sub-left";
  bnOtaBegin(otaHostname);

  if (BN_DEBUG_SERIAL) {
    Serial.print("{\"type\":\"sub_v2_ready\",\"data\":{\"node_id\":");
    Serial.print((uint8_t)SUB_NODE_ID);
    Serial.println(",\"esc_pwm_pin\":10,\"arming_wait_ms\":5000,\"output\":\"neutral\"}}");
  }
}

void loop() {
  bnOtaHandle();
  canBus.poll(millis());
  canOk = canBus.running();
  if (canEpoch != canBus.epoch || !canOk) {
    canEpoch = canBus.epoch;
    armed = false;
    lastValidPropMs = 0;
    faultFlags |= bncan::FAULT_FLAG_COMMAND_TIMEOUT;
    writeEscNeutral();
  }
  // Expire authority before reading newly arrived frames after a long loop gap.
  if (lastValidPropMs != 0 && !propCommandFresh()) {
    armed = false;
    lastValidPropMs = 0;
    faultFlags |= bncan::FAULT_FLAG_COMMAND_TIMEOUT;
    writeEscNeutral();
    canBus.discardPending();
  }

  bncan::CanFrame frame = {};
  for (unsigned count = 0; count < 16 && canReceiveFrame(frame); ++count) {
    handleArmCmd(frame);
    handlePropCmd(frame);
  }

  uint32_t nowMs = millis();
  if (nowMs - lastBatterySampleMs >= BATTERY_SAMPLE_PERIOD_MS) {
    lastBatterySampleMs = nowMs;
    battery.add(analogReadMilliVolts(VBAT_ADC_PIN), nowMs);
  }
  if (lastValidPropMs == 0 || nowMs - lastValidPropMs > activeTimeoutMs) {
    faultFlags |= bncan::FAULT_FLAG_COMMAND_TIMEOUT;
    writeEscNeutral();
  }

  if (!escArmingWaitDone()) {
    writeEscNeutral();
  }

  if (nowMs - lastStatusTxMs >= STATUS_PERIOD_MS) {
    lastStatusTxMs = nowMs;
    sendStatus();
  }

  delay(2);
}
