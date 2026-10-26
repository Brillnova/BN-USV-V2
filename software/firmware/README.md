# V2 firmware

This is prototype firmware. Normal operation and component tests use different sketches; choose the sketch deliberately before uploading.

## Normal operation

- `main_v2_prop_cmd/main_v2_prop_cmd.ino`: Main ESP32-S3 controller.
- `sub_v2_prop_cmd_left/sub_v2_prop_cmd_left.ino`: Left Sub ESP32-C3, CAN node 1.
- `sub_v2_prop_cmd_right/sub_v2_prop_cmd_right.ino`: Right Sub ESP32-C3, CAN node 2.
- `sub_v2_prop_cmd/`: shared Sub implementation included by the Left and Right sketches. Use the dedicated Left/Right sketches to select a node identity.
- `common/`: shared protocol, CAN, control, battery and OTA code used by the sketches.

See the [Main notes](main_v2_prop_cmd/README.md) and [Sub notes](sub_v2_prop_cmd/README.md) for board configuration, wiring and behavior.

## Tests and diagnostics

The following are standalone test sketches, not the normal operating firmware. Uploading one replaces the sketch currently installed on that board. Re-upload the appropriate Main, Left or Right operating sketch after testing.

| Folder | Purpose |
| --- | --- |
| `can_ping_main_test/`, `can_ping_sub_test/` | Check CAN communication between boards. |
| `prop_cmd_codec_test/`, `arm_cmd_codec_test/`, `sub_status_codec_test/` | Check message encoding and decoding without a CAN module. |
| `main_pi_parse_test/` | Check parsing of Pi commands on Main. |
| `sub_esc_pwm_test/` | Check ESC PWM output on a Sub. |
| `tests/` | Compile-time tests for battery monitoring, Main control and CAN recovery logic; these do not require uploading a sketch. |

Test sketches may command outputs. Disconnect propulsion or remove propellers before bench testing. See each test folder's README for its setup and limitations.

## OTA and safety

OTA is disabled by default. To configure it locally, copy `common/secrets_example.h` to `common/secrets.h`, fill in credentials and set an OTA password. Never publish `secrets.h` or use placeholder credentials. Confirm the Arduino ESP32 board package and libraries before building.

Verify physical pin assignments, voltage dividers, RC input voltage, CAN termination, arming and communication-loss behavior on the actual hardware. Bench test before a supervised water test; waypoint autonomy remains experimental and unvalidated.
