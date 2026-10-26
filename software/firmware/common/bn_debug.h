#pragma once

#include <Arduino.h>

#ifndef BN_DEBUG_SERIAL
#define BN_DEBUG_SERIAL 1
#endif

// GCS status is an operational data channel, separate from development-only
// Serial diagnostics.  Keep this enabled whenever the Pi reads Main USB UART.
#ifndef BN_TELEMETRY_SERIAL
#define BN_TELEMETRY_SERIAL 1
#endif

#if BN_DEBUG_SERIAL
#define BN_DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define BN_DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define BN_DEBUG_PRINT(...)
#define BN_DEBUG_PRINTLN(...)
#endif
