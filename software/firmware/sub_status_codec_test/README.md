# SUB_STATUS Codec Test

Standalone protocol test for `SUB_STATUS`.

Purpose:

```text
Verify SUB_STATUS CAN ID generation.
Verify 8-byte payload packing.
Verify Main-side unpacking.
Verify wrong-node frames are ignored.
Verify invalid PWM frames are rejected.
No CAN module is required.
```

Serial Monitor:

```text
Baud: 115200
Line ending: Newline
```

Commands:

```text
test          Run default test cases
run           Same as test
1,1500,1680   Build/decode Left Sub status, 1500 us, 16.80 V
2,1400,1480   Build/decode Right Sub status, 1400 us, 14.80 V
```

Expected CAN IDs:

```text
Left Sub  status = 0x211
Right Sub status = 0x212
```

Voltage encoding:

```text
vbat_mv_div10 = battery voltage in 10 mV units
1680 -> 16.80 V
1480 -> 14.80 V
```

Expected output should include:

```json
"can_id":"0x211"
"unpack_ok":true
"actual_pwm_us":1500
"vbat_mv_div10":1680
"vbat_v":16.80
```

The wrong node case should show:

```json
"case":"wrong_node"
"unpack_ok":false
```

The invalid PWM case should show:

```json
"case":"invalid_pwm"
"unpack_ok":false
```

