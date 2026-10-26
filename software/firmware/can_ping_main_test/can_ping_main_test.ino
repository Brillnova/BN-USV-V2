#include <Arduino.h>
#include <driver/twai.h>

#include "../common/bn_can_protocol.h"

static const gpio_num_t CAN_TX_PIN = GPIO_NUM_7;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_8;
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t PING_PERIOD_MS = 500;

static uint16_t pingSeq = 0;
static uint32_t lastPingMs = 0;
static bool canReady = false;

static esp_err_t canBegin() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  if (err != ESP_OK) return err;

  err = twai_start();
  if (err != ESP_OK) return err;

  return ESP_OK;
}

static bool canSendFrame(const bncan::CanFrame &frame) {
  twai_message_t msg = {};
  msg.identifier = frame.id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = frame.dlc;
  memcpy(msg.data, frame.data, frame.dlc);
  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

static bool canReceiveFrame(bncan::CanFrame &frame) {
  twai_message_t msg = {};
  if (twai_receive(&msg, 0) != ESP_OK) return false;
  if (msg.extd || msg.rtr || msg.data_length_code > 8) return false;
  frame.id = msg.identifier;
  frame.dlc = msg.data_length_code;
  memcpy(frame.data, msg.data, frame.dlc);
  return true;
}

static void sendPing(uint8_t targetNode) {
  bncan::Ping ping = {};
  ping.seq = pingSeq++;
  ping.main_time_ms = millis();

  bncan::CanFrame frame = {};
  bncan::packPing(targetNode, ping, frame);

  bool ok = canSendFrame(frame);
  Serial.print("{\"type\":\"can_ping_tx\",\"data\":{\"target_node\":");
  Serial.print(targetNode);
  Serial.print(",\"can_id\":\"0x");
  Serial.print(frame.id, HEX);
  Serial.print("\",\"seq\":");
  Serial.print(ping.seq);
  Serial.print(",\"tx_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void handleStatusFrame(const bncan::CanFrame &frame) {
  bncan::SubStatus status = {};
  bool leftOk = bncan::unpackSubStatus(frame, bncan::NODE_LEFT_SUB, status);
  bool rightOk = false;
  uint8_t sourceNode = bncan::NODE_LEFT_SUB;

  if (!leftOk) {
    rightOk = bncan::unpackSubStatus(frame, bncan::NODE_RIGHT_SUB, status);
    sourceNode = bncan::NODE_RIGHT_SUB;
  }

  if (!leftOk && !rightOk) return;

  Serial.print("{\"type\":\"can_ping_rx_status\",\"data\":{\"source_node\":");
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("{\"type\":\"can_ping_main_boot\",\"data\":{\"stage\":\"serial_ready\"}}");

  esp_err_t canErr = canBegin();
  canReady = canErr == ESP_OK;
  Serial.print("{\"type\":\"can_ping_main_ready\",\"data\":{\"can_ok\":");
  Serial.print(canReady ? "true" : "false");
  Serial.print(",\"can_err\":");
  Serial.print((int)canErr);
  Serial.println(",\"baud\":115200,\"can_bitrate\":500000}}");
}

void loop() {
  if (!canReady) {
    delay(1000);
    Serial.println("{\"type\":\"can_ping_main_wait\",\"data\":{\"reason\":\"can_not_ready\"}}");
    return;
  }

  uint32_t nowMs = millis();
  if (nowMs - lastPingMs >= PING_PERIOD_MS) {
    lastPingMs = nowMs;
    sendPing(bncan::NODE_LEFT_SUB);
    sendPing(bncan::NODE_RIGHT_SUB);
  }

  bncan::CanFrame frame = {};
  while (canReceiveFrame(frame)) {
    handleStatusFrame(frame);
  }

  delay(2);
}
