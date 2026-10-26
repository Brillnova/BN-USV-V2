# CAN Ping Main Test

Actual CAN bus test for the Main ESP32-S3.

This requires:

```text
Main ESP32-S3
SN65HVD230 CAN module for Main
At least one Sub ESP32-C3 running can_ping_sub_test
SN65HVD230 CAN module for each Sub
Common GND
CANH to CANH
CANL to CANL
```

Main Rev.A pins:

```text
GPIO7 -> CAN TX
GPIO8 -> CAN RX
```

Serial Monitor:

```text
Baud: 115200
Line ending: any
```

Behavior:

```text
Main sends PING to Left Sub  every 500 ms, CAN ID 0x131.
Main sends PING to Right Sub every 500 ms, CAN ID 0x132.
Sub replies with SUB_STATUS, CAN ID 0x211 or 0x212.
```

Expected output:

```json
{"type":"can_ping_tx","data":{"target_node":1,"can_id":"0x131","tx_ok":true}}
{"type":"can_ping_rx_status","data":{"source_node":1,"actual_pwm_us":1500}}
```

Termination:

```text
One Main + one Sub bench test:
Enable 120 ohm on both physical ends if the cable is long enough to matter.
For a very short bench jumper, it may still work with one terminator, but use proper termination for final validation.

Full V2 bus:
Left Sub termination ON
Right Sub termination ON
Main termination OFF
```

