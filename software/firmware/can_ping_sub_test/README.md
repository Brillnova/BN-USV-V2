# CAN Ping Sub Test

Actual CAN bus test for one Sub ESP32-C3.

Target board:

```text
Tenstar Robot ESP32-C3 Super Mini
```

Sub Rev.A pins:

```text
GPIO1 -> CAN TX
GPIO3 -> CAN RX
```

Build identity:

```cpp
#define SUB_NODE_ID bncan::NODE_LEFT_SUB
```

or:

```cpp
#define SUB_NODE_ID bncan::NODE_RIGHT_SUB
```

Default is Left Sub if no build flag is provided.

Behavior:

```text
Sub listens for PING:
Left Sub  listens to CAN ID 0x131
Right Sub listens to CAN ID 0x132

When PING arrives, Sub replies with SUB_STATUS:
Left Sub  replies on CAN ID 0x211
Right Sub replies on CAN ID 0x212
```

Serial Monitor:

```text
Baud: 115200
Line ending: any
```

Expected output when Main sends PING:

```json
{"type":"can_ping_rx","data":{"node_id":1,"ping_seq":0}}
{"type":"can_ping_sub_status_tx","data":{"node_id":1,"event":"ping_reply","can_id":"0x211","tx_ok":true}}
```

Termination:

```text
One Main + one Sub bench test:
Use termination at both bus ends for proper validation.

Full V2 bus:
Left Sub termination ON
Right Sub termination ON
Main termination OFF
```

