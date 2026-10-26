#include "../common/bn_battery_monitor.h"

// Compile-time tests: any failed expectation makes compilation fail.
constexpr bool batteryMonitorTests() {
  bnbattery::Monitor monitor;
  if (monitor.valid(0) || monitor.voltageDiv10(0) != 0) return false;
  for (uint32_t i = 1; i <= 7; ++i) monitor.add(2191, i * 10);
  if (monitor.valid(70)) return false;
  monitor.add(2191, 80);
  if (!monitor.valid(80) || monitor.voltageDiv10(80) != 1680) return false;
  if (!monitor.valid(330) || monitor.valid(331)) return false;
  if (monitor.voltageDiv10(331) != 0) return false;
  monitor.add(0, 340);
  if (monitor.valid(340)) return false;
  for (uint32_t i = 1; i <= 7; ++i) monitor.add(1500, 340 + i * 10);
  if (monitor.valid(410)) return false;
  monitor.add(1500, 420);
  if (monitor.voltageDiv10(420) != 1150) return false;
  // Moving average must evict old samples when the input changes.
  for (uint32_t i = 1; i <= 8; ++i) monitor.add(1800, 420 + i * 10);
  if (monitor.voltageDiv10(500) != 1380) return false;
  monitor.add(2450, 510);
  if (monitor.valid(510) || monitor.voltageDiv10(510) != 0) return false;
  monitor.add(4000, 520);
  if (monitor.valid(520)) return false;
  // Unsigned elapsed time must remain correct across millis() rollover.
  for (uint32_t i = 0; i < 8; ++i) monitor.add(1500, 0xfffffff0u);
  if (!monitor.valid(0x20u) || monitor.voltageDiv10(0x20u) != 1150) return false;
  if (monitor.valid(0x100u)) return false;
  return true;
}

static_assert(batteryMonitorTests(), "Battery conversion/filter/validity test failed");
