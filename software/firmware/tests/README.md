# Firmware Logic Tests

`battery_monitor_test.cpp` evaluates the actual battery monitor at compile time.
It covers startup, 16.8 V conversion, stale data, zero and over-range samples,
recovery, moving-average replacement, and `millis()` rollover.

Run with a C++17 compiler (the installed ESP32 RISC-V compiler also works):

```sh
riscv32-esp-elf-g++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only battery_monitor_test.cpp
```

A zero exit code means all `static_assert` checks passed. No board is required.
These tests do not validate ADC hardware accuracy, wiring, CAN transport timing,
or ESC behavior; follow the Sub sketch's bench verification procedure as well.

`main_control_test.cpp` uses the same compile-time approach for Main RC/Pi
arbitration, override latching, failsafe, PWM mixing, and slew limiting:

```sh
riscv32-esp-elf-g++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only main_control_test.cpp
```

This tests the platform-independent policy used by Main, not interrupt capture
timing or physical receiver/CAN/ESC behavior. Compile the Main sketch for ESP32-S3
as well, then follow its bench verification checklist.

`can_recovery_test.cpp` checks recovery state/action selection, retry gating,
and the Main rearm latch after driver/peer loss:

```sh
riscv32-esp-elf-g++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only can_recovery_test.cpp
```

These compile-time checks do not simulate the TWAI hardware. The power-interruption
acceptance tests in `../CAN_RECOVERY.md` must still be performed on the bench.
