# Sub V2 PROP_CMD Integration Sketch

CAN power-loss recovery is implemented in the shared transport. See
[CAN_RECOVERY.md](../CAN_RECOVERY.md) for diagnostics and power-cycle tests.
Update both Sub identities and Main. Use **USB CDC On Boot = Disabled** for the
currently verified USB-UART connection.

## Unambiguous Left/Right uploads

Open one of these dedicated Arduino IDE sketches instead of editing
`SUB_NODE_ID` by hand:

- `../sub_v2_prop_cmd_left/sub_v2_prop_cmd_left.ino` builds CAN node 1 and
  advertises as `bn-usv-sub-left`.
- `../sub_v2_prop_cmd_right/sub_v2_prop_cmd_right.ino` builds CAN node 2 and
  advertises as `bn-usv-sub-right`.

The OTA hostname is derived from the embedded CAN node identity. Confirm the
hostname in the Arduino IDE network port list before an OTA upload.
After a command timeout or driver interruption, Sub drops ARM authority and
requires a fresh ARM followed by PROP_CMD. Main requires a centered-stick KILL
reset after both Sub links are restored before propulsion can be enabled.

Target:

```text
Tenstar Robot ESP32-C3 Super Mini
```

Purpose:

```text
Receive ARM_CMD over CAN.
Receive PROP_CMD over CAN.
Drive ESC PWM on GPIO10.
Force ESC neutral on boot, timeout, DISARM, KILL, or failsafe.
Send SUB_STATUS at 5 Hz.
```

Sub Rev.A pins:

```text
GPIO1  -> CAN TX
GPIO3  -> CAN RX
GPIO10 -> ESC PWM
GPIO0  -> VBAT ADC (100k / 15k divider, 100nF to GND)
```

Build identity:

```cpp
#define SUB_NODE_ID bncan::NODE_LEFT_SUB
```

or:

```cpp
#define SUB_NODE_ID bncan::NODE_RIGHT_SUB
```

Default is Right Sub if no build flag is provided. Confirm the identity before uploading.

Boot behavior:

```text
First 5000 ms after boot:
ESC output is held at 1500 us neutral.
PROP_CMD may be received, but it is not applied to ESC output yet.
```

Runtime safety:

```text
ARM_CMD ARM + fresh PROP_CMD + arming wait done:
Apply PROP_CMD PWM to ESC.

DISARM / KILL / command timeout / failsafe flag:
Force 1500 us neutral.
```

Serial output examples:

```json
{"type":"prop_cmd_rx","data":{"node_id":1,"pwm_us":1600,"output_allowed":true}}
{"type":"sub_status","data":{"node_id":1,"armed":true,"active_pwm_us":1600}}
```

Serial output policy:

```text
BN_DEBUG_SERIAL=1  Development default. Print USB Serial diagnostics.
BN_DEBUG_SERIAL=0  Production option. Suppress USB Serial diagnostics.
```

## Battery voltage monitoring

GPIO0 is sampled every 10 ms using `analogReadMilliVolts()` with 12-bit
resolution and 11 dB attenuation. An eight-sample moving average is converted
using `(100k + 15k) / 15k` and rounded to the existing CAN 10 mV unit.
Main already forwards this field as `vbat_v`; no CAN layout change is needed.

The divider values match the Sub PCB production BOM (R1/R2). Verify the
assembled board, sense wiring, and measured voltage before water testing.

- Eight valid samples are required at startup and after an invalid sample.
- A zero or >=2450 mV ADC reading invalidates the window immediately.
- Samples older than 250 ms are invalid. Invalid voltage is sent as CAN zero
  (unknown) with `ADC_FAULT`; Sub USB JSON reports `vbat_v: null` and
  `battery_valid: false`. Main currently forwards the CAN sentinel as 0.0 V,
  so consumers must inspect the fault flag rather than interpret it as a measurement.
- ADC errors are diagnostic only in this step and do not stop propulsion.
  `VBAT_OK` remains unset; battery warning/stop thresholds are not implemented.
- A plausible reading does not prove wiring integrity or battery health.
  Open or damaged sense wiring can produce plausible values.

### Bench verification

1. With propulsion disabled, compare the reported voltage against a multimeter
   at several supply voltages within the intended battery operating range.
2. Confirm that 16.8 V produces approximately 2191 mV at GPIO0 and 1680 in
   the CAN voltage field, allowing for resistor and ADC measurement error.
3. Disconnect the battery sense input and check the unknown/ADC fault report;
   reconnect and check recovery after eight valid samples.
4. Recheck boot neutral, DISARM/KILL, and command timeout with telemetry enabled.

ADC API reference: [Espressif Arduino-ESP32 ADC documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html).
