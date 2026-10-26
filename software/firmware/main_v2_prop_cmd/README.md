# Main V2 RC / Pi / CAN Controller

Target: ESP32-S3 USB serial at 115200 baud, CAN at 500 kbps.

## Serial telemetry and diagnostics

`main_status` and CAN-forwarded `sub_status` are the Pi/GCS telemetry channel.
They remain enabled by default through `BN_TELEMETRY_SERIAL=1`, including when
development diagnostics are disabled with `BN_DEBUG_SERIAL=0`. Do not disable
telemetry while the Pi GCS is connected to Main USB serial.

**Power-loss recovery:** See [CAN_RECOVERY.md](../CAN_RECOVERY.md). Both Sub
nodes must be online, and centered sticks with KILL selected clear the propulsion
lock at startup and after an interruption. CAN communication itself recovers
automatically; no all-MCU reset is required by the firmware.

## Rev.A pins

| Signal | GPIO |
| --- | --- |
| RC CH1 steering | 4 |
| RC CH2 throttle | 5 |
| RC CH4 mode | 6 |
| CAN TX | 7 |
| CAN RX | 8 |

The V2 hardware baseline retains the V1 RC mapping. Check the actual assembled
PCB and receiver signal voltage before connecting hardware.

## Control priority

| Condition | Output / control source |
| --- | --- |
| Fresh CH4 >=1700 us | KILL; immediate neutral, CAN KILL |
| Missing or expired RC channel | FAILSAFE; neutral, CAN DISARM |
| CH4 <1300 us | MANUAL; RC throttle and steering, independent of Pi |
| CH4 1300 to <1700 us, override latched | MANUAL; RC control |
| AUTO selected, no override, fresh Pi command | AUTO; Pi control |
| AUTO selected, Pi command expired | FAILSAFE; neutral, CAN DISARM |
| CAN unavailable or rearm lock active | FAILSAFE; neutral (KILL takes priority) |

All three RC channels must have received valid 900-2100 us pulses. CH1/CH2
expire after 300 ms and CH4 after 500 ms. Pi commands expire after 500 ms.
The controller runs at 20 Hz, so timeout detection may take up to one additional
50 ms control interval, plus transport/scheduling latency.

A stick deviation greater than 220 us in AUTO latches MANUAL. Returning sticks
to center does not clear the latch. Select MANUAL, center both sticks within
60 us of neutral, then select AUTO again. Entry into AUTO after startup, KILL,
or RC loss also requires centered sticks; otherwise RC control is selected.
A recovered Pi heartbeat can resume AUTO while RC remains valid and AUTO is selected.

**Behavior change:** Pi-only commands without RC no longer arm propulsion.
There is no RC bypass in this sketch. Use the separate codec/parser test sketches
for tests that do not have a receiver connected.

## Commands and CAN output

Pi USB input retains newline-delimited `cmd_manual` JSON:

```json
{"type":"cmd_manual","timestamp":1.0,"seq":1,"data":{"throttle":0.2,"steering":0.1}}
```

`cmd_manual` is the existing Pi message name; it does not select RC MANUAL mode.
Commands are cached during MANUAL/KILL only after the CAN rearm lock is cleared,
and are applied only under AUTO authority. Lock transitions invalidate cached commands.
Legacy V1 CSV input is not supported. The existing lightweight JSON field parser
is retained; strict JSON/schema and sequence validation remain future work.
Oversized serial lines are discarded through the next newline. Serial and CAN
receive work is bounded per loop to avoid starving control processing.

Throttle/steering use a 20 us deadband and differential mixing:
`left = 1500 + throttle_deviation + steering_deviation`,
`right = 1500 + throttle_deviation - steering_deviation`, clamped to 1000-2000 us.
V1-style slew limiting uses nominal 250 us/s acceleration and 2000 us/s deceleration
(integer PWM steps at 20 Hz). KILL, failsafe, and an all-neutral target bypass slew.

Main sends PROP_CMD every 50 ms and ARM_CMD every 200 ms, also sending ARM_CMD
on mode changes. MANUAL sets RC_AUTHORITY; AUTO sets AUTO_AUTHORITY. Disabled
output sets FAILSAFE_ACTIVE and sends neutral. Sub boot neutral and its own
500 ms command watchdog remain active.

OTA start sends KILL/neutral before transfer and locks Main propulsion until
reboot, including after a failed update. Successful firmware upload restarts Main.

## Telemetry

`main_status` continues to expose `can_ok`, `pi_command_fresh`, `left_pwm_us`,
`right_pwm_us`, and Sub status ages. It now also includes:

- `mode`: MANUAL / AUTO / KILL / FAILSAFE
- `control_source`: RC / PI / KILL / FAILSAFE
- `rc_mode`, `rc_ok`, `manual_latch`
- `ch1_us`, `ch2_us`, `ch4_us`, `cmd_thr_us`, `cmd_str_us`
- `ota_locked`

The existing Pi/GCS reader accepts these fields and can display the control
source and RC pulse values without a protocol adapter. PWM fields are Main's
commanded outputs; Sub `actual_pwm_us` must be checked separately.
`BN_DEBUG_SERIAL=0` suppresses development diagnostics while GCS telemetry
continues through `BN_TELEMETRY_SERIAL=1`.

## Main 2S battery telemetry

Main samples the XT60 auxiliary sense lead on GPIO9 through the on-board
100k/15k divider. `main_status` publishes `vbat_v` and `battery_valid`; the
Pi GCS displays this Main ADC value directly and does not require INA226.

## Verification

Compile with the board's verified serial setting. The current Pi connection
uses the USB-UART bridge, so use USB CDC disabled:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=disabled main_v2_prop_cmd
```

Compile-time controller tests are in `../tests/main_control_test.cpp`. They cover
startup without RC, mode priority, override persistence and release, AUTO entry,
RC/Pi expiry, CAN unavailability, timer rollover, mixing, deadband, and slew.

Bench sequence (propulsion disabled while checking commands):

1. Boot without RC; fresh Pi commands must leave Main in FAILSAFE at neutral.
2. Connect both Subs, supply CH1/CH2/CH4, center sticks and select KILL until
   `can_rearm_required=false`. Select MANUAL and check RC mixing without Pi.
3. Center sticks, select AUTO, and send Pi commands at 10-20 Hz.
4. Move either stick beyond 220 us; verify RC takeover stays latched after centering.
5. Select MANUAL, center, and select AUTO; verify return to Pi control.
6. Select KILL while commanding thrust; verify neutral and Sub disarm.
7. Stop Pi commands in AUTO; verify neutral after expiry. MANUAL must still work.
8. Remove each RC PWM input; verify FAILSAFE. Check non-neutral AUTO re-entry.
9. Check OTA start/failure and verify propulsion remains disabled until reboot.
10. Verify actual Sub outputs, boot neutral, and CAN timeout on hardware.

Receiver radio-loss behavior requires a separate test: a receiver may continue
emitting valid held PWM after losing radio contact. Configure receiver failsafe
to request KILL and verify it; pulse-age checks alone cannot detect held values.
CAN bus-off recovery and Sub status-loss locking are implemented; physical power-loss
verification is pending. Full Sub fault supervision, battery shutdown, and on-water
verification remain pending. `can_ok` reports a RUNNING controller, not confirmed
peer health. ADC hardware testing is deferred until PCB arrival.
