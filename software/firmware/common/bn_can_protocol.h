#pragma once

#include <Arduino.h>

namespace bncan {

static const uint8_t NODE_MAIN = 0x00;
static const uint8_t NODE_LEFT_SUB = 0x01;
static const uint8_t NODE_RIGHT_SUB = 0x02;
static const uint8_t NODE_BROADCAST = 0x7F;
static const uint8_t CAN_TARGET_BROADCAST = 0x0F;

static const uint32_t CAN_ID_PROP_CMD_BASE = 0x110;
static const uint32_t CAN_ID_ARM_CMD_BASE = 0x120;
static const uint32_t CAN_ID_PING_BASE = 0x130;
static const uint32_t CAN_ID_SUB_STATUS_BASE = 0x210;

static const uint16_t PWM_MIN_US = 1000;
static const uint16_t PWM_NEUTRAL_US = 1500;
static const uint16_t PWM_MAX_US = 2000;

static const uint8_t PROP_FLAG_ARM_ALLOWED = 1u << 0;
static const uint8_t PROP_FLAG_FAILSAFE_ACTIVE = 1u << 1;
static const uint8_t PROP_FLAG_RC_AUTHORITY = 1u << 2;
static const uint8_t PROP_FLAG_AUTO_AUTHORITY = 1u << 3;

static const uint8_t STATUS_FLAG_ARMED = 1u << 0;
static const uint8_t STATUS_FLAG_MAIN_FRESH = 1u << 1;
static const uint8_t STATUS_FLAG_ESC_OUTPUT_ENABLED = 1u << 2;
static const uint8_t STATUS_FLAG_LOCAL_5V_OK = 1u << 3;
static const uint8_t STATUS_FLAG_AUX_5V_PRESENT = 1u << 4;
static const uint8_t STATUS_FLAG_VBAT_OK = 1u << 5;
static const uint8_t STATUS_FLAG_CAN_OK = 1u << 6;

static const uint8_t FAULT_FLAG_COMMAND_TIMEOUT = 1u << 0;
static const uint8_t FAULT_FLAG_BATTERY_LOW = 1u << 1;
static const uint8_t FAULT_FLAG_ADC_FAULT = 1u << 2;
static const uint8_t FAULT_FLAG_ESC_SIGNAL_FAULT = 1u << 3;
static const uint8_t FAULT_FLAG_CAN_RX_ERROR = 1u << 4;
static const uint8_t FAULT_FLAG_BOOT_CONFIG_FAULT = 1u << 5;
static const uint8_t FAULT_FLAG_OVERTEMP_RESERVED = 1u << 6;
static const uint8_t FAULT_FLAG_CRITICAL_FAULT = 1u << 7;

static const uint8_t ARM_STATE_DISARM = 0;
static const uint8_t ARM_STATE_ARM = 1;
static const uint8_t ARM_STATE_KILL = 2;

static const uint8_t ARM_REASON_NORMAL = 0;
static const uint8_t ARM_REASON_OPERATOR_DISARM = 1;
static const uint8_t ARM_REASON_RC_LOST = 2;
static const uint8_t ARM_REASON_PI_LOST = 3;
static const uint8_t ARM_REASON_SUB_FAULT = 4;
static const uint8_t ARM_REASON_BATTERY_LOW = 5;
static const uint8_t ARM_REASON_STARTUP = 6;

struct CanFrame {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
};

struct PropCmd {
  uint16_t seq;
  uint16_t pwm_us;
  uint8_t flags;
  uint8_t timeout_100ms;
};

struct SubStatus {
  uint16_t seq;
  uint16_t actual_pwm_us;
  uint16_t vbat_mv_div10;
  uint8_t status_flags;
  uint8_t fault_flags;
};

struct ArmCmd {
  uint16_t seq;
  uint8_t arm_state;
  uint8_t reason_code;
};

struct Ping {
  uint16_t seq;
  uint32_t main_time_ms;
};

static inline uint32_t propCmdId(uint8_t target_node) {
  return CAN_ID_PROP_CMD_BASE + target_node;
}

static inline uint32_t subStatusId(uint8_t source_node) {
  return CAN_ID_SUB_STATUS_BASE + source_node;
}

static inline uint32_t armCmdId(uint8_t target_node) {
  return CAN_ID_ARM_CMD_BASE + target_node;
}

static inline uint32_t pingId(uint8_t target_node) {
  return CAN_ID_PING_BASE + target_node;
}

static inline uint8_t checksumXor7(const uint8_t data[8]) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

static inline void putU16LE(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static inline uint16_t getU16LE(const uint8_t *src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static inline void putU32LE(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8) & 0xFFu);
  dst[2] = (uint8_t)((value >> 16) & 0xFFu);
  dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static inline uint32_t getU32LE(const uint8_t *src) {
  return (uint32_t)src[0] |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

static inline uint16_t clampPwm(uint16_t pwm_us) {
  if (pwm_us < PWM_MIN_US) return PWM_MIN_US;
  if (pwm_us > PWM_MAX_US) return PWM_MAX_US;
  return pwm_us;
}

static inline void packPropCmd(uint8_t target_node, const PropCmd &cmd, CanFrame &frame) {
  frame.id = propCmdId(target_node);
  frame.dlc = 8;
  putU16LE(&frame.data[0], cmd.seq);
  putU16LE(&frame.data[2], clampPwm(cmd.pwm_us));
  frame.data[4] = cmd.flags & 0x0Fu;
  frame.data[5] = cmd.timeout_100ms;
  frame.data[6] = 0;
  frame.data[7] = checksumXor7(frame.data);
}

static inline bool unpackPropCmd(const CanFrame &frame, uint8_t this_node, PropCmd &cmd) {
  if (frame.id != propCmdId(this_node)) return false;
  if (frame.dlc != 8) return false;
  if (frame.data[7] != checksumXor7(frame.data)) return false;

  uint16_t pwm = getU16LE(&frame.data[2]);
  if (pwm < PWM_MIN_US || pwm > PWM_MAX_US) return false;

  cmd.seq = getU16LE(&frame.data[0]);
  cmd.pwm_us = pwm;
  cmd.flags = frame.data[4];
  cmd.timeout_100ms = frame.data[5];
  return true;
}

static inline bool propCmdAllowsOutput(const PropCmd &cmd) {
  bool armed = (cmd.flags & PROP_FLAG_ARM_ALLOWED) != 0;
  bool failsafe = (cmd.flags & PROP_FLAG_FAILSAFE_ACTIVE) != 0;
  return armed && !failsafe;
}

static inline void packSubStatus(uint8_t source_node, const SubStatus &status, CanFrame &frame) {
  frame.id = subStatusId(source_node);
  frame.dlc = 8;
  putU16LE(&frame.data[0], status.seq);
  putU16LE(&frame.data[2], clampPwm(status.actual_pwm_us));
  putU16LE(&frame.data[4], status.vbat_mv_div10);
  frame.data[6] = status.status_flags;
  frame.data[7] = status.fault_flags;
}

static inline bool unpackSubStatus(const CanFrame &frame, uint8_t source_node, SubStatus &status) {
  if (frame.id != subStatusId(source_node)) return false;
  if (frame.dlc != 8) return false;

  uint16_t pwm = getU16LE(&frame.data[2]);
  if (pwm < PWM_MIN_US || pwm > PWM_MAX_US) return false;

  status.seq = getU16LE(&frame.data[0]);
  status.actual_pwm_us = pwm;
  status.vbat_mv_div10 = getU16LE(&frame.data[4]);
  status.status_flags = frame.data[6];
  status.fault_flags = frame.data[7];
  return true;
}

static inline bool isValidArmState(uint8_t arm_state) {
  return arm_state == ARM_STATE_DISARM ||
         arm_state == ARM_STATE_ARM ||
         arm_state == ARM_STATE_KILL;
}

static inline void packArmCmd(uint8_t target_node, const ArmCmd &cmd, CanFrame &frame) {
  frame.id = armCmdId(target_node);
  frame.dlc = 8;
  putU16LE(&frame.data[0], cmd.seq);
  frame.data[2] = cmd.arm_state;
  frame.data[3] = cmd.reason_code;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = checksumXor7(frame.data);
}

static inline bool unpackArmCmd(const CanFrame &frame, uint8_t this_node, ArmCmd &cmd) {
  bool targeted = frame.id == armCmdId(this_node);
  bool broadcast = frame.id == armCmdId(CAN_TARGET_BROADCAST);
  if (!targeted && !broadcast) return false;
  if (frame.dlc != 8) return false;
  if (frame.data[7] != checksumXor7(frame.data)) return false;

  uint8_t arm_state = frame.data[2];
  if (!isValidArmState(arm_state)) return false;

  cmd.seq = getU16LE(&frame.data[0]);
  cmd.arm_state = arm_state;
  cmd.reason_code = frame.data[3];
  return true;
}

static inline void packPing(uint8_t target_node, const Ping &ping, CanFrame &frame) {
  frame.id = pingId(target_node);
  frame.dlc = 8;
  putU16LE(&frame.data[0], ping.seq);
  putU32LE(&frame.data[2], ping.main_time_ms);
  frame.data[6] = 0;
  frame.data[7] = checksumXor7(frame.data);
}

static inline bool unpackPing(const CanFrame &frame, uint8_t this_node, Ping &ping) {
  bool targeted = frame.id == pingId(this_node);
  bool broadcast = frame.id == pingId(CAN_TARGET_BROADCAST);
  if (!targeted && !broadcast) return false;
  if (frame.dlc != 8) return false;
  if (frame.data[7] != checksumXor7(frame.data)) return false;

  ping.seq = getU16LE(&frame.data[0]);
  ping.main_time_ms = getU32LE(&frame.data[2]);
  return true;
}

}  // namespace bncan
