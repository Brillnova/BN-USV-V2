# V2 CAN Power-Loss Recovery

Applies to `main_v2_prop_cmd` and both identities of `sub_v2_prop_cmd`.
Update all three MCUs; no CAN frame layout or bitrate changes are required.
USB CDC On Boot must be enabled for native USB diagnostics.

## Transport behavior

- Poll TWAI state/alerts every 20 ms without waiting in a recovery loop.
- BUS_OFF initiates `twai_initiate_recovery()`. After hardware recovery reaches
  STOPPED, explicitly call `twai_start()`; recovery alone does not restart CAN.
- Retry installation/start/recovery at most once per second after a failed attempt.
- Recovery clears TX; start clears RX. Recovered control uses new frames.
- Use single-shot transmissions, so an unacknowledged old propulsion command is
  not retried indefinitely. The normal 20 Hz command / 5 Hz status schedule sends
  new data. Arbitration loss or a bus error can drop a frame; the next scheduled
  frame is the retry. Multi-frame delivery is not atomic.
- A pending TX with no success for 500 ms triggers a stop and subsequent restart,
  subject to the retry interval. This aborts the hardware TX as well as the queue.
- Queue submission is non-blocking. A full queue is discarded on submission failure.
  Persistent wiring faults or a bus held dominant must be fixed physically; software
  cannot guarantee a recovery time while the electrical bus is unusable.

Reference: [Espressif TWAI driver states and recovery API](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/twai.html).

## Main safety and operator workflow

Communication restarts automatically. Propulsion is locked at boot and after a
driver interruption or loss of either required Sub. Both Left and Right must
report status within 1000 ms and both must report fresh Main commands. A Sub
reboot/timeout reported through a cleared MAIN_FRESH bit also locks propulsion.
The status watchdog is necessary because another surviving node can still ACK
CAN frames when one Sub loses power.

Main continues to send neutral/DISARM (or KILL when selected) while locked, so
Sub freshness can recover without enabling propulsion. Once communication is
restored, center both RC sticks (within 60 us), select KILL, and wait for:

```text
can_state: RUNNING
can_peers_fresh: true
can_rearm_required: false
```

Then select MANUAL or AUTO. AUTO also requires a newly received Pi command;
cached Pi commands are invalidated on lock transitions and are ignored while
locked. Holding KILL with centered sticks during reconnection satisfies the reset
condition, but propulsion remains in KILL until the switch is moved.

One-Sub bench connections still exchange status, but cannot unlock Main propulsion.
Peer-loss detection takes up to 1000 ms plus the control interval. A very brief
power interruption may not be detected by the status watchdog; Sub's own boot
neutral and command timeout rules still apply. This is not a verified on-water
safety certification or a full Sub fault supervisor.

## Sub behavior

Any controller interruption or command timeout forces neutral and clears ARM
authority. A recovery start clears old receive data; a command-timeout transition
also discards pending queues before accepting new frames. A fresh ARM_CMD and
subsequent PROP_CMD are required for output. The five-second boot-neutral rule
remains in effect. ADC diagnostics do not prevent CAN recovery.

## Diagnostics

Main `main_status` and Sub USB `sub_status` include:

| Field | Meaning |
| --- | --- |
| `can_state` | RUNNING / BUS_OFF / RECOVERING / STOPPED / UNAVAILABLE |
| `can_recoveries` | Successfully initiated bus-off recoveries |
| `can_stall_restarts` | Stops requested for stalled pending TX |
| `can_tx_error_counter`, `can_rx_error_counter` | Latest sampled controller error counters |
| `can_tx_failed_count` | Driver transmission-failure counter |
| `can_tx_success_events` | Polls observing TX_SUCCESS; alerts can coalesce |
| `can_queue_failures` | Failed software submission attempts |
| `can_last_tx_error`, `can_last_driver_error` | Last API return codes |

`can_ok` means the controller is currently RUNNING, not that all peers are present.
Sub `can_tx_ok` is retained for compatibility and means queue acceptance only.
Check received peer status and increasing success events for actual bus activity.
Diagnostics counters are local and may reset on MCU reboot/driver reinstall.

## Bench acceptance (pending physical verification)

With propulsion disconnected, repeat each case without rebooting the other MCUs:

1. Boot Main first, then both Subs. Verify both source_node 1/2 messages arrive.
2. Remove and restore Left power while Main/Right stay powered; repeat for Right.
3. Remove and restore Main power while both Subs stay powered.
4. Power off both Subs for at least 10 seconds; restore them.
5. Disconnect and reconnect one transceiver's power and then the CAN cable.
6. Verify status exchange recovers, `can_rearm_required` stays true until the
   centered-stick KILL reset, and no old thrust command is applied on reconnect.
7. Clear the reset, verify MANUAL control, then repeat the failure and recovery.
8. Repeat failures several times and capture state/counter logs if recovery stalls.

Keep grounds and the intended two 120-ohm terminations intact. Verify actual ESC
output separately; Main's commanded PWM alone is not physical output validation.
