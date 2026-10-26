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

static void printPropCmd(const bncan::PropCmd &cmd) {
  Serial.print("\"seq\":");
  Serial.print(cmd.seq);
  Serial.print(",\"pwm_us\":");
  Serial.print(cmd.pwm_us);
  Serial.print(",\"flags\":");
  Serial.print(cmd.flags);
  Serial.print(",\"timeout_100ms\":");
  Serial.print(cmd.timeout_100ms);
  Serial.print(",\"output_allowed\":");
  Serial.print(bncan::propCmdAllowsOutput(cmd) ? "true" : "false");
}

static void runCase(uint8_t node, uint16_t pwmUs, uint8_t flags, uint8_t timeout100ms) {
  bncan::PropCmd tx = {};
  tx.seq = seq++;
  tx.pwm_us = pwmUs;
  tx.flags = flags;
  tx.timeout_100ms = timeout100ms;

  bncan::CanFrame frame = {};
  bncan::packPropCmd(node, tx, frame);

  bncan::PropCmd rx = {};
  bool ok = bncan::unpackPropCmd(frame, node, rx);

  Serial.print("{\"type\":\"prop_cmd_codec_test\",\"data\":{");
  Serial.print("\"target_node\":");
  Serial.print(node);
  Serial.print(",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  if (ok) {
    Serial.print(",\"decoded\":{");
    printPropCmd(rx);
    Serial.print("}");
  }
  Serial.println("}}");
}

static void runCorruptChecksumCase() {
  bncan::PropCmd tx = {};
  tx.seq = seq++;
  tx.pwm_us = 1600;
  tx.flags = bncan::PROP_FLAG_ARM_ALLOWED | bncan::PROP_FLAG_AUTO_AUTHORITY;
  tx.timeout_100ms = 5;

  bncan::CanFrame frame = {};
  bncan::packPropCmd(bncan::NODE_LEFT_SUB, tx, frame);
  frame.data[7] ^= 0x55;

  bncan::PropCmd rx = {};
  bool ok = bncan::unpackPropCmd(frame, bncan::NODE_LEFT_SUB, rx);

  Serial.print("{\"type\":\"prop_cmd_codec_test\",\"data\":{");
  Serial.print("\"case\":\"corrupt_checksum\",");
  printFrame(frame);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void runWrongNodeCase() {
  bncan::PropCmd tx = {};
  tx.seq = seq++;
  tx.pwm_us = 1600;
  tx.flags = bncan::PROP_FLAG_ARM_ALLOWED | bncan::PROP_FLAG_AUTO_AUTHORITY;
  tx.timeout_100ms = 5;

  bncan::CanFrame frame = {};
  bncan::packPropCmd(bncan::NODE_LEFT_SUB, tx, frame);

  bncan::PropCmd rx = {};
  bool ok = bncan::unpackPropCmd(frame, bncan::NODE_RIGHT_SUB, rx);

  Serial.print("{\"type\":\"prop_cmd_codec_test\",\"data\":{");
  Serial.print("\"case\":\"wrong_node\",");
  printFrame(frame);
  Serial.print(",\"receiver_node\":");
  Serial.print(bncan::NODE_RIGHT_SUB);
  Serial.print(",\"unpack_ok\":");
  Serial.print(ok ? "true" : "false");
  Serial.println("}}");
}

static void runDefaultTests() {
  runCase(bncan::NODE_LEFT_SUB, 1500, bncan::PROP_FLAG_ARM_ALLOWED | bncan::PROP_FLAG_AUTO_AUTHORITY, 5);
  runCase(bncan::NODE_LEFT_SUB, 1600, bncan::PROP_FLAG_ARM_ALLOWED | bncan::PROP_FLAG_AUTO_AUTHORITY, 5);
  runCase(bncan::NODE_RIGHT_SUB, 1400, bncan::PROP_FLAG_ARM_ALLOWED | bncan::PROP_FLAG_RC_AUTHORITY, 5);
  runCase(bncan::NODE_RIGHT_SUB, 1500, bncan::PROP_FLAG_FAILSAFE_ACTIVE, 5);
  runCorruptChecksumCase();
  runWrongNodeCase();
}

static bool parseTwoInts(const String &line, int &a, int &b) {
  int comma = line.indexOf(',');
  if (comma < 0) return false;

  String first = line.substring(0, comma);
  String second = line.substring(comma + 1);
  first.trim();
  second.trim();

  if (first.length() == 0 || second.length() == 0) return false;
  a = first.toInt();
  b = second.toInt();
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
  if (parseTwoInts(line, node, pwm)) {
    if (node != bncan::NODE_LEFT_SUB && node != bncan::NODE_RIGHT_SUB) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_NODE\"}}");
      return;
    }
    if (pwm < bncan::PWM_MIN_US || pwm > bncan::PWM_MAX_US) {
      Serial.println("{\"type\":\"error\",\"data\":{\"code\":\"INVALID_PWM\"}}");
      return;
    }

    runCase((uint8_t)node, (uint16_t)pwm, bncan::PROP_FLAG_ARM_ALLOWED | bncan::PROP_FLAG_AUTO_AUTHORITY, 5);
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

  Serial.println("{\"type\":\"prop_cmd_codec_test_ready\",\"data\":{\"baud\":115200}}");
  runDefaultTests();
}

void loop() {
  readSerialLines();
  delay(5);
}

