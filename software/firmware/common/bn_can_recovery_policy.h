#pragma once
#include <stdint.h>

namespace bnrecovery {
enum class State { Missing, Stopped, Running, BusOff, Recovering };
enum class Action { None, Install, Start, Recover, Restart };
constexpr Action action(State state, bool retryDue, bool stalled = false) {
  if (!retryDue) return Action::None;
  return state == State::Missing ? Action::Install :
      state == State::Stopped ? Action::Start :
      state == State::BusOff ? Action::Recover :
      state == State::Running && stalled ? Action::Restart : Action::None;
}

// Communication resumes automatically; propulsion requires an operator reset.
class MainGuard {
 public:
  constexpr void trip() { locked_ = true; }
  constexpr bool update(bool driverRunning, bool peersFresh, bool rcKillCentered) {
    if (!driverRunning || !peersFresh) locked_ = true;
    else if (rcKillCentered) locked_ = false;
    return !locked_;
  }
  constexpr bool locked() const { return locked_; }
 private:
  bool locked_ = true;
};
}  // namespace bnrecovery
