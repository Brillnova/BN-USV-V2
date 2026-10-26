#pragma once
#include <stdint.h>

namespace bncontrol {
enum class Mode { Manual, Auto, Kill, Failsafe };
enum class Reason { Normal, RcLost, PiLost, CanUnavailable, Kill };

struct Channel {
  uint16_t us = 1500;
  uint32_t lastMs = 0;
  bool seen = false;
};
struct Input {
  Channel steering, throttle, mode;
  uint16_t piThrottle = 1500, piSteering = 1500;
  uint32_t piLastMs = 0;
  bool piSeen = false, canReady = false;
};
struct Output {
  Mode mode = Mode::Failsafe, rcMode = Mode::Failsafe;
  Reason reason = Reason::RcLost;
  bool rcOk = false, piFresh = false, manualLatch = false;
  uint16_t throttle = 1500, steering = 1500;
  uint16_t left = 1500, right = 1500;
  constexpr bool enabled() const { return mode == Mode::Manual || mode == Mode::Auto; }
};
constexpr int magnitude(int v) { return v < 0 ? -v : v; }
constexpr uint16_t clamp(int v) { return v < 1000 ? 1000 : (v > 2000 ? 2000 : v); }
constexpr int deadband(int v) { return magnitude(v) <= 20 ? 0 : v; }
constexpr bool fresh(const Channel &c, uint32_t now, uint32_t timeout) {
  return c.seen && c.us >= 900 && c.us <= 2100 && uint32_t(now - c.lastMs) <= timeout;
}
constexpr Mode modeFromPulse(uint16_t us) {
  return us < 1300 ? Mode::Manual : (us < 1700 ? Mode::Auto : Mode::Kill);
}
inline const char *name(Mode mode) {
  switch (mode) {
    case Mode::Manual: return "MANUAL";
    case Mode::Auto: return "AUTO";
    case Mode::Kill: return "KILL";
    default: return "FAILSAFE";
  }
}

class Controller {
 public:
  constexpr Output update(const Input &in, uint32_t now) {
    Output out;
    out.piFresh = in.piSeen && uint32_t(now - in.piLastMs) <= 500;
    out.rcOk = fresh(in.steering, now, 300) && fresh(in.throttle, now, 300) && fresh(in.mode, now, 500);
    out.rcMode = fresh(in.mode, now, 500) ? modeFromPulse(in.mode.us) : Mode::Failsafe;
    uint32_t dt = started_ ? uint32_t(now - lastMs_) : 0;
    if (dt > 100) dt = 100;  // No large acceleration jump after a delayed loop.
    lastMs_ = now;
    started_ = true;

    if (out.rcMode == Mode::Kill) {
      latch_ = false;
      lastRc_ = Mode::Kill;
      out.mode = Mode::Kill;
      out.reason = Reason::Kill;
    } else if (!out.rcOk) {
      lastRc_ = Mode::Failsafe;
      out.reason = Reason::RcLost;
    } else {
      const int thrDev = int(in.throttle.us) - 1500;
      const int strDev = int(in.steering.us) - 1500;
      if (out.rcMode == Mode::Manual) latch_ = false;
      if (out.rcMode == Mode::Auto) {
        if (lastRc_ != Mode::Auto) {
          latch_ = magnitude(thrDev) >= 60 || magnitude(strDev) >= 60;
        }
        if (magnitude(thrDev) > 220 || magnitude(strDev) > 220) latch_ = true;
      }
      lastRc_ = out.rcMode;
      out.mode = latch_ ? Mode::Manual : out.rcMode;
      out.reason = Reason::Normal;
      if (out.mode == Mode::Manual) {
        out.throttle = clamp(in.throttle.us);
        out.steering = clamp(in.steering.us);
      } else if (out.piFresh) {
        out.throttle = clamp(in.piThrottle);
        out.steering = clamp(in.piSteering);
      } else {
        out.mode = Mode::Failsafe;
        out.reason = Reason::PiLost;
      }
    }
    if (!in.canReady && out.mode != Mode::Kill) {
      out.mode = Mode::Failsafe;
      out.reason = Reason::CanUnavailable;
    }
    out.manualLatch = latch_;
    int thr = deadband(int(out.throttle) - 1500);
    int str = deadband(int(out.steering) - 1500);
    uint16_t targetL = clamp(1500 + thr + str);
    uint16_t targetR = clamp(1500 + thr - str);
    if (!out.enabled() || (targetL == 1500 && targetR == 1500)) {
      left_ = right_ = 1500;
      if (!out.enabled()) out.throttle = out.steering = 1500;
    } else {
      left_ = slew(left_, targetL, dt);
      right_ = slew(right_, targetR, dt);
    }
    out.left = left_;
    out.right = right_;
    return out;
  }

 private:
  static constexpr uint16_t slew(uint16_t current, uint16_t target, uint32_t dt) {
    if (dt == 0) return current;
    const int rate = magnitude(int(target) - 1500) > magnitude(int(current) - 1500) ? 250 : 2000;
    int step = int((rate * dt + 500) / 1000);
    if (step < 1) step = 1;
    int diff = int(target) - current;
    return magnitude(diff) <= step ? target : uint16_t(int(current) + (diff > 0 ? step : -step));
  }
  bool latch_ = false, started_ = false;
  Mode lastRc_ = Mode::Manual;
  uint32_t lastMs_ = 0;
  uint16_t left_ = 1500, right_ = 1500;
};
}  // namespace bncontrol
