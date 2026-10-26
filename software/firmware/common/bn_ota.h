#pragma once

#include <Arduino.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef ENABLE_OTA
#define ENABLE_OTA 0
#endif

#if ENABLE_OTA
#include <ArduinoOTA.h>
#include <WiFi.h>

#ifndef BN_OTA_CONNECT_TIMEOUT_MS
#define BN_OTA_CONNECT_TIMEOUT_MS 5000
#endif

static inline bool bnOtaBegin(const char *hostname) {
#if defined(BN_WIFI_SSID) && defined(BN_WIFI_PASSWORD)
  WiFi.mode(WIFI_STA);
  WiFi.begin(BN_WIFI_SSID, BN_WIFI_PASSWORD);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < BN_OTA_CONNECT_TIMEOUT_MS) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  ArduinoOTA.setHostname(hostname);
#if defined(BN_OTA_PASSWORD)
  ArduinoOTA.setPassword(BN_OTA_PASSWORD);
#endif
  ArduinoOTA.begin();
  return true;
#else
  (void)hostname;
  return false;
#endif
}

static inline void bnOtaHandle() {
  ArduinoOTA.handle();
}

#else

static inline bool bnOtaBegin(const char *hostname) {
  (void)hostname;
  return false;
}

static inline void bnOtaHandle() {
}

#endif

