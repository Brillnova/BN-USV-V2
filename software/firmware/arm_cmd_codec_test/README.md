# ARM_CMD Codec Test

Standalone protocol test for `ARM_CMD`.

No CAN module is required.

Serial Monitor:

```text
Baud: 115200
Line ending: Newline
```

Commands:

```text
test      Run default test cases
run       Same as test
1,1,0     Left Sub ARM, normal reason
1,0,1     Left Sub DISARM, operator_disarm reason
2,2,1     Right Sub KILL, operator_disarm reason
15,0,2    Broadcast DISARM, rc_lost reason
```

Target values:

```text
1  = Left Sub
2  = Right Sub
15 = Broadcast, CAN ID 0x12F
```

ARM states:

```text
0 = DISARM
1 = ARM
2 = KILL
```

Expected CAN IDs:

```text
Left Sub  ARM_CMD = 0x121
Right Sub ARM_CMD = 0x122
Broadcast ARM_CMD = 0x12F
```

The wrong-node, corrupt-checksum, and invalid-state cases should report
`"unpack_ok":false`.

