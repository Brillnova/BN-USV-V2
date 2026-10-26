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

static void runCase(uint8_t targetNode, uint8_t receiverNode, uint8_t armState, uint8_t reasonCode) {
  bncan::ArmCmd tx = {};
  tx.seq = seq++;
  tx.arm_state = armState;
  tx.reason_code = reasonCode;

  bncan::CanFrame frame = {};
  bncan::packArmCmd(targetNode, tx, frame);

  bncan::ArmCmd rx = {};
  bool ok = bncan::unpackArmCmd(frame, receiverNode, rx);

  Serial.print("{\"type\":\"arm_cmd_codec_test\",\"data\":{");
  Serial.print("\"target_node\":");
  Serial.print(targetNode);
  Serial.print(",\"receiver_node\":");
  Serial.print(receiverNode);
  Serial.print(",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  if (ok) {
    Serial.print(",\"decoded\":{\"seq\":");
    Serial.print(rx.seq);
    Serial.print(",\"arm_state\":");
    Serial.print(rx.arm_state);
    Serial.print(",\"reason_code\":");
    Serial.print(rx.reason_code);
    Serial.print("}");
  }
  Serial.println("}}");
}

static void runCorruptChecksumCase() {
  bncan::ArmCmd tx = {};
  tx.seq = seq++;
  tx.arm_state = bncan::ARM_STATE_ARM;
  tx.reason_code = bncan::ARM_REASON_NORMAL;

  bncan::CanFrame frame = {};
  bncan::packArmCmd(bncan::NODE_LEFT_SUB, tx, frame);
  frame.data[7] ^= 0x55;

  bncan::ArmCmd rx = {};
  bool ok = bncan::unpackArmCmd(frame, bncan::NODE_LEFT_SUB, rx);

  Serial.print("{\"type\":\"arm_cmd_codec_test\",\"data\":{\"case\":\"corrupt_checksum\",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void runInvalidStateCase() {
  bncan::ArmCmd tx = {};
  tx.seq = seq++;
  tx.arm_state = 99;
  tx.reason_code = bncan::ARM_REASON_NORMAL;

  bncan::CanFrame frame = {};
  bncan::packArmCmd(bncan::NODE_LEFT_SUB, tx, frame);

  bncan::ArmCmd rx = {};
  bool ok = bncan::unpackArmCmd(frame, bncan::NODE_LEFT_SUB, rx);

  Serial.print("{\"type\":\"arm_cmd_codec_test\",\"data\":{\"case\":\"invalid_arm_state\",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void runDefaultTests() {
  runCase(bncan::NODE_LEFT_SUB, bncan::NODE_LEFT_SUB, bncan::ARM_STATE_DISARM, bncan::ARM_REASON_STARTUP);
  runCase(bncan::NODE_LEFT_SUB, bncan::NODE_LEFT_SUB, bncan::ARM_STATE_ARM, bncan::ARM_REASON_NORMAL);
  runCase(bncan::NODE_RIGHT_SUB, bncan::NODE_RIGHT_SUB, bncan::ARM_STATE_KILL, bncan::ARM_REASON_OPERATOR_DISARM);
  runCase(bncan::CAN_TARGET_BROADCAST, bncan::NODE_LEFT_SUB, bncan::ARM_STATE_DISARM, bncan::ARM_REASON_RC_LOST);
  runCase(bncan::CAN_TARGET_BROADCAST, bncan::NODE_RIGHT_SUB, bncan::ARM_STATE_DISARM, bncan::ARM_REASON_RC_LOST);
  runCase(bncan::NODE_LEFT_SUB, bncan::NODE_RIGHT_SUB, bncan::ARM_STATE_ARM, bncan::ARM_REASON_NORMAL);
  runCorruptChecksumCase();
  runInvalidStateCase();
}

static bool parseThreeInts(const String &line, int &target, int &state, int &reason) {
  int comma1 = line.indexOf(',');
  if (comma1 < 0) return false;
  int comma2 = line.indexOf(',', comma1 + 1);
  if (comma2 < 0) return false;

  target = line.substring(0, comma1).toInt();
  state = line.substring(comma1 + 1, comma2).toInt();
  reason = line.substring(comma2 + 1).toInt();
  return true;
}

static void handleCommand(String line) {
  line.trim();
  line.toLowerCase();

  if (line == "test" || line == "run") {
    runDefaultTests();
    return;
  }

  int target = 0;
  int state = 0;
  int reason = 0;
  if (parseThreeInts(line, target, state, reason)) {
    if (target != bncan::NODE_LEFT_SUB &&
        target != bncan::NODE_RIGHT_SUB &&
        target != bncan::CAN_TARGET_BROADCAST) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_TARGET\"}}");
      return;
    }
    if (!bncan::isValidArmState((uint8_t)state)) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_ARM_STATE\"}}");
      return;
    }

    uint8_t receiver = target == bncan::CAN_TARGET_BROADCAST ? bncan::NODE_LEFT_SUB : (uint8_t)target;
    runCase((uint8_t)target, receiver, (uint8_t)state, (uint8_t)reason);
    return;
  }

  Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_COMMAND\"}}");
}

static void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxLine.length() > 0) handleCommand(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() < 80) rxLine += c;
      else {
        rxLine = "";
        Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"LINE_TOO_LONG\"}}");
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("{\"type\":\"arm_cmd_codec_test_ready\",\"data\":{\"baud\":115200}}");
  runDefaultTests();
}

void loop() {
  readSerialLines();
  delay(5);
}

