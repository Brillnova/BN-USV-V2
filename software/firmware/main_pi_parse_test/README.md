# Main Pi Command Parse Test

Standalone bench-test firmware for Main ESP32-S3.

Purpose:

```text
PC acts as Raspberry Pi over USB Serial.
Main receives cmd_manual JSON.
Main parses throttle/steering.
Main mixes left/right PWM values.
Main prints JSON ack/status.
No CAN module is required for this test.
```

Serial Monitor:

```text
Baud: 115200
Line ending: Newline
```

Example commands:

```json
{"type":"cmd_manual","timestamp":1.0,"seq":1,"data":{"throttle":0.0,"steering":0.0}}
{"type":"cmd_manual","timestamp":1.1,"seq":2,"data":{"throttle":0.2,"steering":0.1}}
{"type":"cmd_manual","timestamp":1.2,"seq":3,"data":{"throttle":0.2,"steering":-0.1}}
{"type":"cmd_manual","timestamp":1.3,"seq":4,"data":{"throttle":-0.2,"steering":0.0}}
```

Expected mix examples:

```text
throttle 0.0, steering 0.0  -> left 1500, right 1500
throttle 0.2, steering 0.1  -> left 1650, right 1550
throttle 0.2, steering -0.1 -> left 1550, right 1650
throttle -0.2, steering 0.0 -> left 1400, right 1400
```

Timeout behavior:

```text
If no valid cmd_manual arrives for 500 ms:
left_pwm  = 1500
right_pwm = 1500
pi_fresh  = false
```

Invalid input example:

```json
{"type":"cmd_manual","timestamp":1.4,"seq":5,"data":{"throttle":1.5,"steering":0.0}}
```

Expected response:

```json
{"type":"ack","seq":-1,"data":{"accepted":false,"reason":"PARSE_ERROR"}}
```

OTA:

OTA follows the shared `firmware/common/bn_ota.h` policy. It is disabled
by default unless `ENABLE_OTA` is set to 1 in `firmware/common/secrets.h`
or a build flag.

