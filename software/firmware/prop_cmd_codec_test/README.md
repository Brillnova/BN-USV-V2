# PROP_CMD Codec Test

Standalone protocol test for `PROP_CMD`.

Purpose:

```text
Verify PROP_CMD CAN ID generation.
Verify 8-byte payload packing.
Verify checksum generation.
Verify Sub-side unpacking.
Verify wrong-node frames are ignored.
Verify corrupt checksum frames are rejected.
No CAN module is required.
```

Serial Monitor:

```text
Baud: 115200
Line ending: Newline
```

Commands:

```text
test       Run default test cases
run        Same as test
1,1500     Build/decode Left Sub PROP_CMD at 1500 us
1,1600     Build/decode Left Sub PROP_CMD at 1600 us
2,1400     Build/decode Right Sub PROP_CMD at 1400 us
```

Expected CAN IDs:

```text
Left Sub  = 0x111
Right Sub = 0x112
```

Expected payload example:

```text
node=1, pwm=1500, flags=9, timeout=5
```

The output should show:

```json
"can_id":"0x111"
"unpack_ok":true
"pwm_us":1500
"output_allowed":true
```

The corrupt checksum case should show:

```json
"case":"corrupt_checksum"
"unpack_ok":false
```

The wrong node case should show:

```json
"case":"wrong_node"
"unpack_ok":false
```

