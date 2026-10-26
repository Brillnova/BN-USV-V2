#include <Arduino.h>

#include "../common/bn_can_protocol.h"

static const uint32_t SERIAL_BAUD = 115200;
static uint16_t seq = 0;
static String rxLine;

static void printFrame(const bncan::CanFrame &frame) {
  Serial.print("\"can_id\":\"0x");
  Serial.print(frame.id, HEX);
  Serial.print("\",\"dlc\":");
  Serial.print(frame.dlc);
  Serial.print(",\"data\":[");

  for (uint8_t i = 0; i < frame.dlc; i++) {
    if (i > 0) Serial.print(",");
    Serial.print(frame.data[i]);
  }

  Serial.print("]");
}

static void printSubStatus(const bncan::SubStatus &status) {
  Serial.print("\"seq\":");
  Serial.print(status.seq);
  Serial.print(",\"actual_pwm_us\":");
  Serial.print(status.actual_pwm_us);
  Serial.print(",\"vbat_mv_div10\":");
  Serial.print(status.vbat_mv_div10);
  Serial.print(",\"vbat_v\":");
  Serial.print(status.vbat_mv_div10 / 100.0f, 2);
  Serial.print(",\"status_flags\":");
  Serial.print(status.status_flags);
  Serial.print(",\"fault_flags\":");
  Serial.print(status.fault_flags);
}

static void runCase(uint8_t node, uint16_t pwmUs, uint16_t vbatMvDiv10, uint8_t statusFlags, uint8_t faultFlags) {
  bncan::SubStatus tx = {};
  tx.seq = seq++;
  tx.actual_pwm_us = pwmUs;
  tx.vbat_mv_div10 = vbatMvDiv10;
  tx.status_flags = statusFlags;
  tx.fault_flags = faultFlags;

  bncan::CanFrame frame = {};
  bncan::packSubStatus(node, tx, frame);

  bncan::SubStatus rx = {};
  bool ok = bncan::unpackSubStatus(frame, node, rx);

  Serial.print("{\"type\":\"sub_status_codec_test\",\"data\":{");
  Serial.print("\"source_node\":");
  Serial.print(node);
  Serial.print(",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  if (ok) {
    Serial.print(",\"decoded\":{");
    printSubStatus(rx);
    Serial.print("}");
  }
  Serial.println("}}");
}

static void runWrongNodeCase() {
  bncan::SubStatus tx = {};
  tx.seq = seq++;
  tx.actual_pwm_us = 1500;
  tx.vbat_mv_div10 = 1680;
  tx.status_flags = bncan::STATUS_FLAG_ARMED | bncan::STATUS_FLAG_MAIN_FRESH |
                    bncan::STATUS_FLAG_ESC_OUTPUT_ENABLED | bncan::STATUS_FLAG_VBAT_OK |
                    bncan::STATUS_FLAG_CAN_OK;
  tx.fault_flags = 0;

  bncan::CanFrame frame = {};
  bncan::packSubStatus(bncan::NODE_LEFT_SUB, tx, frame);

  bncan::SubStatus rx = {};
  bool ok = bncan::unpackSubStatus(frame, bncan::NODE_RIGHT_SUB, rx);

  Serial.print("{\"type\":\"sub_status_codec_test\",\"data\":{");
  Serial.print("\"case\":\"wrong_node\",");
  printFrame(frame);
  Serial.print(",\"expected_source_node\":");
  Serial.print(bncan::NODE_RIGHT_SUB);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void runInvalidPwmCase() {
  bncan::SubStatus tx = {};
  tx.seq = seq++;
  tx.actual_pwm_us = 1500;
  tx.vbat_mv_div10 = 1480;
  tx.status_flags = bncan::STATUS_FLAG_MAIN_FRESH | bncan::STATUS_FLAG_CAN_OK;
  tx.fault_flags = bncan::FAULT_FLAG_COMMAND_TIMEOUT;

  bncan::CanFrame frame = {};
  bncan::packSubStatus(bncan::NODE_LEFT_SUB, tx, frame);
  bncan::putU16LE(&frame.data[2], 2500);

  bncan::SubStatus rx = {};
  bool ok = bncan::unpackSubStatus(frame, bncan::NODE_LEFT_SUB, rx);

  Serial.print("{\"type\":\"sub_status_codec_test\",\"data\":{");
  Serial.print("\"case\":\"invalid_pwm\",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void runDefaultTests() {
  runCase(
      bncan::NODE_LEFT_SUB,
      1500,
      1680,
      bncan::STATUS_FLAG_ARMED | bncan::STATUS_FLAG_MAIN_FRESH |
          bncan::STATUS_FLAG_ESC_OUTPUT_ENABLED | bncan::STATUS_FLAG_LOCAL_5V_OK |
          bncan::STATUS_FLAG_VBAT_OK | bncan::STATUS_FLAG_CAN_OK,
      0);

  runCase(
      bncan::NODE_RIGHT_SUB,
      1400,
      1480,
      bncan::STATUS_FLAG_MAIN_FRESH | bncan::STATUS_FLAG_LOCAL_5V_OK |
          bncan::STATUS_FLAG_VBAT_OK | bncan::STATUS_FLAG_CAN_OK,
      bncan::FAULT_FLAG_COMMAND_TIMEOUT);

  runCase(
      bncan::NODE_RIGHT_SUB,
      1500,
      1320,
      bncan::STATUS_FLAG_LOCAL_5V_OK | bncan::STATUS_FLAG_CAN_OK,
      bncan::FAULT_FLAG_BATTERY_LOW | bncan::FAULT_FLAG_CRITICAL_FAULT);

  runWrongNodeCase();
  runInvalidPwmCase();
}

static bool parseThreeInts(const String &line, int &node, int &pwm, int &vbat) {
  int comma1 = line.indexOf(',');
  if (comma1 < 0) return false;
  int comma2 = line.indexOf(',', comma1 + 1);
  if (comma2 < 0) return false;

  String nodeText = line.substring(0, comma1);
  String pwmText = line.substring(comma1 + 1, comma2);
  String vbatText = line.substring(comma2 + 1);
  nodeText.trim();
  pwmText.trim();
  vbatText.trim();

  if (nodeText.length() == 0 || pwmText.length() == 0 || vbatText.length() == 0) return false;

  node = nodeText.toInt();
  pwm = pwmText.toInt();
  vbat = vbatText.toInt();
  return true;
}

static void handleCommand(String line) {
  line.trim();
  line.toLowerCase();

  if (line == "test" || line == "run") {
    runDefaultTests();
    return;
  }

  int node = 0;
  int pwm = 0;
  int vbat = 0;
  if (parseThreeInts(line, node, pwm, vbat)) {
    if (node != bncan::NODE_LEFT_SUB && node != bncan::NODE_RIGHT_SUB) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_NODE\"}}");
      return;
    }
    if (pwm < bncan::PWM_MIN_US || pwm > bncan::PWM_MAX_US) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_PWM\"}}");
      return;
    }
    if (vbat < 0 || vbat > 65535) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_VBAT\"}}");
      return;
    }

    runCase(
        (uint8_t)node,
        (uint16_t)pwm,
        (uint16_t)vbat,
        bncan::STATUS_FLAG_MAIN_FRESH | bncan::STATUS_FLAG_LOCAL_5V_OK |
            bncan::STATUS_FLAG_VBAT_OK | bncan::STATUS_FLAG_CAN_OK,
        0);
    return;
  }

  Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_COMMAND\"}}");
}

static void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxLine.length() > 0) {
        handleCommand(rxLine);
      }
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

  Serial.println("{\"type\":\"sub_status_codec_test_ready\",\"data\":{\"baud\":115200}}");
  runDefaultTests();
}

void loop() {
  readSerialLines();
  delay(5);
}

