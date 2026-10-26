#include "../common/bn_can_recovery_policy.h"
using namespace bnrecovery;
static_assert(action(State::Missing, true) == Action::Install, "retry initial failure");
static_assert(action(State::BusOff, true) == Action::Recover, "initiate recovery");
static_assert(action(State::Recovering, true) == Action::None, "wait without blocking");
static_assert(action(State::Stopped, true) == Action::Start, "restart after bus recovery");
static_assert(action(State::Running, true) == Action::None, "do not restart healthy bus");
static_assert(action(State::Running, true, true) == Action::Restart, "purge stalled hardware TX");
static_assert(action(State::BusOff, false) == Action::None, "rate limit retry");
static_assert(action(State::Stopped, false) == Action::None, "rate limit start");
static_assert(action(State::Running, false, true) == Action::None, "rate limit stall restart");
constexpr bool guardTest() {
  MainGuard g;
  if (!g.locked() || g.update(true, true, false)) return false;
  if (g.update(false, true, true) || g.update(true, false, true)) return false;
  if (!g.update(true, true, true)) return false;
  if (!g.update(true, true, false)) return false;
  if (g.update(true, false, false)) return false; // One peer disappears, even if bus remains running.
  if (g.update(true, true, false)) return false; // Restored communications must not resume thrust.
  if (!g.update(true, true, true)) return false;
  g.trip();
  return g.locked() && !g.update(true, true, false);
}
static_assert(guardTest(), "rearm gate must survive power loss and recovery");
