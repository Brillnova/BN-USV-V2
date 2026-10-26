#include "../common/bn_main_control.h"
using namespace bncontrol;

constexpr Input rc(uint32_t now, uint16_t mode = 1500, uint16_t thr = 1500, uint16_t str = 1500) {
  Input in;
  in.steering = {str, now, true}; in.throttle = {thr, now, true}; in.mode = {mode, now, true};
  in.canReady = true; in.piSeen = true; in.piLastMs = now; in.piThrottle = 1800;
  return in;
}
constexpr bool startupAndPriority() {
  Controller c;
  Input in;
  in.canReady = true; in.piSeen = true;
  auto out = c.update(in, 0);
  if (out.enabled() || out.rcOk || out.left != 1500) return false;
  in = rc(50);
  out = c.update(in, 50);
  if (out.mode != Mode::Auto || out.left <= 1500 || out.left > 1513) return false;
  in = rc(100, 1000, 1700); in.piSeen = false;
  out = c.update(in, 100);
  if (out.mode != Mode::Manual || out.throttle != 1700) return false;
  in = rc(150, 2000, 2000); in.steering.seen = false;
  out = c.update(in, 150);
  return out.mode == Mode::Kill && out.left == 1500 && out.right == 1500;
}
constexpr bool overrideLatch() {
  Controller c;
  c.update(rc(0), 0);
  auto out = c.update(rc(50, 1500, 1500, 1800), 50);
  if (out.mode != Mode::Manual || !out.manualLatch) return false;
  out = c.update(rc(100), 100);
  if (out.mode != Mode::Manual || !out.manualLatch) return false;
  c.update(rc(150, 1000), 150);
  out = c.update(rc(200), 200);
  if (out.mode != Mode::Auto || out.manualLatch) return false;
  c.update(rc(250, 2000), 250);
  out = c.update(rc(300, 1500, 1560), 300);
  return out.mode == Mode::Manual && out.manualLatch;
}
constexpr bool timeouts() {
  Controller c;
  auto in = rc(0);
  c.update(in, 0);
  if (!c.update(in, 300).rcOk) return false;
  auto out = c.update(in, 301);
  if (out.mode != Mode::Failsafe || out.reason != Reason::RcLost || out.left != 1500) return false;
  in = rc(500); in.piLastMs = 0;
  if (c.update(in, 500).mode != Mode::Auto) return false;
  in = rc(501); in.piLastMs = 0;
  out = c.update(in, 501);
  if (out.mode != Mode::Failsafe || out.reason != Reason::PiLost || out.left != 1500) return false;
  in = rc(550, 1000); in.mode.lastMs = 0;
  if (c.update(in, 550).rcOk) return false;
  in = rc(600); in.canReady = false;
  out = c.update(in, 600);
  if (out.enabled() || out.reason != Reason::CanUnavailable) return false;
  in = rc(0xfffffff0u);
  return c.update(in, 0x20).mode == Mode::Auto;
}
constexpr bool mixingAndSlew() {
  Controller c;
  c.update(rc(0, 1000), 0);
  auto out = c.update(rc(50, 1000, 2000, 2000), 50);
  if (out.left != 1513 || out.right != 1500) return false;
  out = c.update(rc(100, 1000, 1500, 1500), 100);
  if (out.left != 1500 || out.right != 1500) return false;
  out = c.update(rc(150, 1000, 1520, 1480), 150);
  if (out.left != 1500 || out.right != 1500) return false;
  for (uint32_t now = 200; now <= 5000; now += 50) out = c.update(rc(now, 1000, 900, 2100), now);
  return out.left == 1500 && out.right == 1000;
}
static_assert(startupAndPriority(), "Startup/control priority failed");
static_assert(overrideLatch(), "Override latch/re-entry failed");
static_assert(timeouts(), "Timeout/CAN failure/rollover failed");
static_assert(mixingAndSlew(), "Mix/deadband/clamp/slew failed");
