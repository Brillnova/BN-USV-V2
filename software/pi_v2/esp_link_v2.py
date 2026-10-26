import json
import time
from dataclasses import dataclass, field
from typing import Any

import serial


@dataclass
class MainTelemetry:
    main_status: dict[str, Any] = field(default_factory=dict)
    left_sub_status: dict[str, Any] = field(default_factory=dict)
    right_sub_status: dict[str, Any] = field(default_factory=dict)
    raw_lines: list[str] = field(default_factory=list)


class EspMainLink:
    """USB serial link from Raspberry Pi to the BN-USV V2 Main ESP32-S3."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200, timeout: float = 0.02):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.seq = 0
        self.telemetry = MainTelemetry()
        self._rx_buffer = bytearray()
        self.opened_at = time.monotonic()
        self.last_main_status_at: float | None = None

    def close(self) -> None:
        self.serial.close()

    def send_manual_command(self, throttle: float, steering: float) -> None:
        throttle = self._clamp_unit(throttle)
        steering = self._clamp_unit(steering)

        message = {
            "type": "cmd_manual",
            "timestamp": time.time(),
            "seq": self.seq,
            "data": {
                "throttle": throttle,
                "steering": steering,
            },
        }
        self.seq += 1

        line = json.dumps(message, separators=(",", ":")) + "\n"
        self.serial.write(line.encode("utf-8"))

    def send_neutral(self) -> None:
        self.send_manual_command(0.0, 0.0)

    def poll(self) -> MainTelemetry:
        # A Main status line is longer than the 10 ms serial timeout. readline()
        # can therefore return a partial JSON fragment, which makes GCS lose all
        # Main telemetry while shorter Sub messages happen to survive. Accumulate
        # non-blocking reads until a full newline-delimited message is present.
        available = self.serial.in_waiting
        if available > 0:
            self._rx_buffer.extend(self.serial.read(available))

        while True:
            newline = self._rx_buffer.find(b"\n")
            if newline < 0:
                break
            payload = bytes(self._rx_buffer[:newline])
            del self._rx_buffer[:newline + 1]
            self._handle_line(payload.decode("utf-8", errors="replace").strip())

        # A malformed device stream must not grow memory without bound.
        if len(self._rx_buffer) > 8192:
            self._rx_buffer.clear()

        return self.telemetry

    def _handle_line(self, line: str) -> None:
        if not line:
            return

        self.telemetry.raw_lines.append(line)
        if len(self.telemetry.raw_lines) > 100:
            self.telemetry.raw_lines = self.telemetry.raw_lines[-100:]

        self._parse_line(line)

    def _parse_line(self, line: str) -> None:
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            return

        msg_type = message.get("type")
        data = message.get("data", {})

        if msg_type == "main_status":
            self.telemetry.main_status = data
            self.last_main_status_at = time.monotonic()
        elif msg_type == "telemetry_status":
            self.telemetry.main_status = self._normalize_v1_status(data)
            self.last_main_status_at = time.monotonic()
        elif msg_type == "sub_status":
            source_node = data.get("source_node")
            if source_node == 1:
                self.telemetry.left_sub_status = data
            elif source_node == 2:
                self.telemetry.right_sub_status = data

    @staticmethod
    def _normalize_v1_status(data: dict[str, Any]) -> dict[str, Any]:
        status = dict(data)
        status["protocol"] = "v1"
        status["control_source"] = data.get("mode", "")
        status["pi_command_fresh"] = data.get("pi_fresh")
        status["left_pwm_us"] = data.get("left_pwm")
        status["right_pwm_us"] = data.get("right_pwm")
        status["rc_steering"] = EspMainLink._pwm_to_unit(data.get("ch1_us"))
        status["rc_throttle"] = EspMainLink._pwm_to_unit(data.get("ch2_us"))
        return status

    @staticmethod
    def _pwm_to_unit(value: Any) -> float | None:
        if not isinstance(value, (int, float)):
            return None

        normalized = (float(value) - 1500.0) / 500.0
        return EspMainLink._clamp_unit(normalized)

    @staticmethod
    def _clamp_unit(value: float) -> float:
        if value < -1.0:
            return -1.0
        if value > 1.0:
            return 1.0
        return value
