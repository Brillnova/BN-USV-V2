# Sub ESC PWM Test

Standalone bench-test firmware for one Sub MCU.

Target board:

```text
Tenstar Robot ESP32-C3 Super Mini
```

Signal wiring:

```text
Tenstar GPIO10 -> ESC PWM signal
Tenstar GND    -> ESC signal GND
```

Power wiring depends on the bench setup. Keep logic GND and ESC signal
GND common.

Safety:

```text
Remove propeller before testing.
Boot output is always 1500 us neutral.
Timeout output is always 1500 us neutral.
```

Serial Monitor:

```text
Baud: 115200
Line ending: Newline
```

Commands:

```text
arm       Enable PWM commands, output remains neutral first
1500      Set neutral
1600      Example low forward command
1000      Minimum command
2000      Maximum command
neutral   Force neutral
status    Print status
disarm    Disable output and force neutral
kill      Same as disarm
stop      Same as disarm
```

If no valid command arrives for 3000 ms while armed, the firmware
automatically disarms and returns to 1500 us neutral.

OTA:

OTA follows the shared `firmware/common/bn_ota.h` policy. It is disabled
by default unless `ENABLE_OTA` is set to 1 in `firmware/common/secrets.h`
or a build flag.

