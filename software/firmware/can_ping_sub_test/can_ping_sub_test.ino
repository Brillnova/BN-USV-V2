#include <Arduino.h>
#include <driver/twai.h>

#include "../common/bn_can_protocol.h"

#ifndef SUB_NODE_ID
#define SUB_NODE_ID bncan::NODE_LEFT_SUB
#endif

static const gpio_num_t CAN_TX_PIN = GPIO_NUM_1;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_3;
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t STATUS_HEARTBEAT_MS = 1000;

static uint16_t statusSeq = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastPingMs = 0;

static bool canBegin() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
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

static bool sendStatus(const char *event) {
  bncan::SubStatus status = {};
  status.seq = statusSeq++;
  status.actual_pwm_us = bncan::PWM_NEUTRAL_US;
  status.vbat_mv_div10 = 0;
  status.status_flags = bncan::STATUS_FLAG_CAN_OK;
  if (lastPingMs != 0 && millis() - lastPingMs <= 1000) {
    status.status_flags |= bncan::STATUS_FLAG_MAIN_FRESH;
  }
  status.fault_flags = 0;

  bncan::CanFrame frame = {};
  bncan::packSubStatus((uint8_t)SUB_NODE_ID, status, frame);
  bool ok = canSendFrame(frame);

  Serial.print("{\"type\":\"can_ping_sub_status_tx\",\"data\":{\"node_id\":");
  Serial.print((uint8_t)SUB_NODE_ID);
  Serial.print(",\"event\":\"");
  Serial.print(event);
  Serial.print("\",\"can_id\":\"0x");
  Serial.print(frame.id, HEX);
  Serial.print("\",\"seq\":");
  Serial.print(status.seq);
  Serial.print(",\"tx_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
  return ok;
}

static void handlePingFrame(const bncan::CanFrame &frame) {
  bncan::Ping ping = {};
  if (!bncan::unpackPing(frame, (uint8_t)SUB_NODE_ID, ping)) return;

  lastPingMs = millis();

  Serial.print("{\"type\":\"can_ping_rx\",\"data\":{\"node_id\":");
  Serial.print((uint8_t)SUB_NODE_ID);
  Serial.print(",\"ping_seq\":");
  Serial.print(ping.seq);
  Serial.print(",\"main_time_ms\":");
  Serial.print(ping.main_time_ms);
  Serial.println("}}");

  sendStatus("ping_reply");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  bool ok = canBegin();
  Serial.print("{\"type\":\"can_ping_sub_ready\",\"data\":{\"node_id\":");
  Serial.print((uint8_t)SUB_NODE_ID);
  Serial.print(",\"can_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println(",\"baud\":115200,\"can_bitrate\":500000}}");
}

void loop() {
  bncan::CanFrame frame = {};
  while (canReceiveFrame(frame)) {
    handlePingFrame(frame);
  }

  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_HEARTBEAT_MS) {
    lastStatusMs = nowMs;
    sendStatus("heartbeat");
  }

  delay(2);
}

