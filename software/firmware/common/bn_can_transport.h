#pragma once
#include <Arduino.h>
#include <driver/twai.h>
#include "bn_can_protocol.h"
#include "bn_can_recovery_policy.h"

namespace bntransport {
class Bus {
 public:
  Bus(gpio_num_t tx, gpio_num_t rx) : tx_(tx), rx_(rx) {}
  esp_err_t begin() {
    if (!installed_) {
      twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(tx_, rx_, TWAI_MODE_NORMAL);
      general.tx_queue_len = 8;
      general.rx_queue_len = 16;
      general.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF;
      twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
      twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
      lastError = twai_driver_install(&general, &timing, &filter);
      if (lastError != ESP_OK) return lastError;
      installed_ = true;
    }
    lastError = twai_start();  // Clears RX; stopped/recovery has already cleared TX.
    running_ = lastError == ESP_OK;
    return lastError;
  }

  void poll(uint32_t now) {
    if (polled_ && uint32_t(now - lastPoll_) < 20) return;
    polled_ = true;
    lastPoll_ = now;
    uint32_t alerts = 0;
    if (installed_ && twai_read_alerts(&alerts, 0) == ESP_OK) {
      if (alerts & TWAI_ALERT_TX_SUCCESS) ++txSuccessEvents;
      if (alerts & TWAI_ALERT_TX_FAILED) ++txFailedEvents;
    }
    twai_status_info_t info = {};
    bool haveStatus = installed_ && twai_get_status_info(&info) == ESP_OK;
    state_ = !haveStatus ? bnrecovery::State::Missing :
        info.state == TWAI_STATE_RUNNING ? bnrecovery::State::Running :
        info.state == TWAI_STATE_BUS_OFF ? bnrecovery::State::BusOff :
        info.state == TWAI_STATE_RECOVERING ? bnrecovery::State::Recovering : bnrecovery::State::Stopped;
    if (haveStatus) status = info;
    if (!haveStatus || info.msgs_to_tx == 0 || (alerts & TWAI_ALERT_TX_SUCCESS)) {
      pending_ = false;
    } else if (!pending_) {
      pending_ = true;
      pendingSince_ = now;
    }
    if (running_ && state_ != bnrecovery::State::Running) ++epoch;
    running_ = state_ == bnrecovery::State::Running;
    const bool due = !attempted_ || uint32_t(now - lastAttempt_) >= 1000;
    const auto next = bnrecovery::action(state_, due, pending_ && uint32_t(now - pendingSince_) >= 500);
    if (next == bnrecovery::Action::None) return;
    attempted_ = true;
    lastAttempt_ = now;
    if (next == bnrecovery::Action::Restart) {
      lastError = twai_stop(); // Aborts hardware TX as well as clearing the queue.
      if (lastError == ESP_OK) {
        running_ = false;
        state_ = bnrecovery::State::Stopped;
        ++epoch;
        ++stallRestarts;
      }
    } else if (next == bnrecovery::Action::Recover) {
      lastError = twai_initiate_recovery(); // Clears pending TX and waits for bus idle.
      if (lastError == ESP_OK) { ++recoveries; state_ = bnrecovery::State::Recovering; }
    } else {
      if (!haveStatus) installed_ = false;
      lastError = begin();
      if (lastError == ESP_OK) { ++epoch; state_ = bnrecovery::State::Running; }
    }
  }
  bool running() const { return running_; }
  const char *stateName() const {
    switch (state_) {
      case bnrecovery::State::Running: return "RUNNING";
      case bnrecovery::State::Stopped: return "STOPPED";
      case bnrecovery::State::BusOff: return "BUS_OFF";
      case bnrecovery::State::Recovering: return "RECOVERING";
      default: return "UNAVAILABLE";
    }
  }
  bool send(const bncan::CanFrame &frame) {
    if (!running_) return false;
    twai_message_t msg = {};
    msg.identifier = frame.id;
    msg.ss = 1; // Do not retain/retransmit an old command indefinitely without ACK.
    msg.data_length_code = frame.dlc;
    memcpy(msg.data, frame.data, frame.dlc);
    lastTxError = twai_transmit(&msg, 0);
    if (lastTxError != ESP_OK) { ++queueFailures; twai_clear_transmit_queue(); }
    return lastTxError == ESP_OK;
  }
  bool receive(bncan::CanFrame &frame) {
    if (!running_) return false;
    twai_message_t msg = {};
    if (twai_receive(&msg, 0) != ESP_OK) return false;
    if (msg.extd || msg.rtr || msg.data_length_code > 8) return false;
    frame.id = msg.identifier;
    frame.dlc = msg.data_length_code;
    memcpy(frame.data, msg.data, frame.dlc);
    return true;
  }
  void discardPending() {
    if (installed_) { twai_clear_transmit_queue(); twai_clear_receive_queue(); }
  }
  // Append fields inside an existing JSON data object. Alerts are event counts,
  // not exact frame counts (multiple alerts can coalesce between polls).
  void printDiagnostics() const {
    Serial.print(",\"can_state\":\""); Serial.print(stateName());
    Serial.print("\",\"can_recoveries\":"); Serial.print(recoveries);
    Serial.print(",\"can_tx_error_counter\":"); Serial.print(status.tx_error_counter);
    Serial.print(",\"can_rx_error_counter\":"); Serial.print(status.rx_error_counter);
    Serial.print(",\"can_tx_failed_count\":"); Serial.print(status.tx_failed_count);
    Serial.print(",\"can_tx_success_events\":"); Serial.print(txSuccessEvents);
    Serial.print(",\"can_queue_failures\":"); Serial.print(queueFailures);
    Serial.print(",\"can_stall_restarts\":"); Serial.print(stallRestarts);
    Serial.print(",\"can_last_tx_error\":"); Serial.print((int)lastTxError);
    Serial.print(",\"can_last_driver_error\":"); Serial.print((int)lastError);
  }
  uint32_t epoch = 0, recoveries = 0, txSuccessEvents = 0, txFailedEvents = 0, queueFailures = 0;
  uint32_t stallRestarts = 0;
  esp_err_t lastError = ESP_OK, lastTxError = ESP_OK;
  twai_status_info_t status = {};
 private:
  gpio_num_t tx_, rx_;
  bool installed_ = false, running_ = false, polled_ = false, attempted_ = false;
  uint32_t lastPoll_ = 0, lastAttempt_ = 0;
  bool pending_ = false;
  uint32_t pendingSince_ = 0;
  bnrecovery::State state_ = bnrecovery::State::Missing;
};
}  // namespace bntransport
