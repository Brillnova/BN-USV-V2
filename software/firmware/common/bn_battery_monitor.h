#pragma once

#include <stdint.h>

namespace bnbattery {

// Main/Sub Rev.A: R1 = 100k (VBAT to ADC), R2 = 15k (ADC to GND).
// Use calibrated ADC millivolts, not raw counts scaled by 3.3 V.
class Monitor {
 public:
  static const uint8_t SAMPLE_COUNT = 8;
  static const uint32_t STALE_MS = 250;

  constexpr void add(uint32_t adcMv, uint32_t nowMs) {
    lastSampleMs_ = nowMs;
    // Conservative upper margin below the C3's 2500 mV measurement limit.
    // Zero includes a missing battery/sense lead; the cause is not identifiable.
    if (adcMv == 0 || adcMv >= 2450) {
      count_ = 0;
      index_ = 0;
      sum_ = 0;
      return;
    }
    if (count_ == SAMPLE_COUNT) sum_ -= samples_[index_];
    else ++count_;
    samples_[index_] = adcMv;
    sum_ += adcMv;
    index_ = (index_ + 1) % SAMPLE_COUNT;
  }

  constexpr bool valid(uint32_t nowMs) const {
    return count_ == SAMPLE_COUNT && uint32_t(nowMs - lastSampleMs_) <= STALE_MS;
  }

  constexpr uint16_t voltageDiv10(uint32_t nowMs) const {
    if (!valid(nowMs)) return 0;  // Protocol sentinel: unknown, not a measured 0 V.
    // Battery mV = ADC mV * 115 / 15; CAN unit = 10 mV, rounded.
    const uint32_t denominator = SAMPLE_COUNT * 150u;
    return uint16_t((sum_ * 115u + denominator / 2u) / denominator);
  }

 private:
  uint32_t samples_[SAMPLE_COUNT] = {};
  uint32_t sum_ = 0;
  uint32_t lastSampleMs_ = 0;
  uint8_t count_ = 0;
  uint8_t index_ = 0;
};

}  // namespace bnbattery
