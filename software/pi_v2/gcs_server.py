from __future__ import annotations

import argparse
import json
import mimetypes
import os
import threading
import time
from contextlib import suppress
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sensors_v2 import SensorConfig


STATIC_DIR = os.path.join(os.path.dirname(__file__), "gcs_static")
DEFAULT_MISSION_FILE = os.path.join(os.path.dirname(__file__), "mission_plan.json")
DEFAULT_TEST_LOG_DIR = os.path.join(os.path.dirname(__file__), "test_logs")


class GcsState:
    def __init__(self, esp_port: str, esp_baud: int, command_rate_hz: float, sensor_config: SensorConfig | None,
                 demo: bool = False, mission_file: str | None = None):
        self.lock = threading.Lock()
        self.esp_port = esp_port
        self.esp_baud = esp_baud
        self.command_period = 1.0 / command_rate_hz
        self.sensor_config = sensor_config
        self.demo = demo
        self.throttle = 0.0
        self.steering = 0.0
        self.enabled = False
        self.last_client_command = 0.0
        self.last_esp_error = ""
        self.last_esp_connect = 0.0
        self.mission_file = mission_file
        self.active_waypoints, self.mission_revision = self._load_mission()
        self.waypoints = list(self.active_waypoints)
        from mission_controller import MissionController
        self.mission_controller = MissionController()
        self.telemetry: dict[str, Any] = {
            "main": {},
            "left": {},
            "right": {},
            "raw": [],
        }
        self.sensors: dict[str, Any] = {
            "gps": {},
            "imu": {},
            "mag": {},
            "battery": {},
        }
        self.stop_event = threading.Event()
        self.last_esp_status = 0.0
        self.last_sent_throttle = 0.0
        self.last_sent_steering = 0.0
        self._last_rc_state: tuple[Any, ...] | None = None
        self._last_can_state: tuple[Any, ...] | None = None
        from test_logger import TestLogger
        self.test_logger = TestLogger(DEFAULT_TEST_LOG_DIR)

    def set_command(self, throttle: float, steering: float, enabled: bool) -> None:
        with self.lock:
            if self.mission_controller.status == "RUNNING":
                self.mission_controller.pause("Manual Output selected; mission paused.")
            self.throttle = self._clamp_unit(throttle)
            self.steering = self._clamp_unit(steering)
            self.enabled = enabled
            self.last_client_command = time.time()

    def stop_output(self) -> None:
        with self.lock:
            self.mission_controller.stop("STOP selected; mission stopped.")
            self.throttle = 0.0
            self.steering = 0.0
            self.enabled = False
            self.last_client_command = time.time()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            age = time.time() - self.last_client_command if self.last_client_command else None
            main_age = time.monotonic() - self.last_esp_status if self.last_esp_status else None
            return {
                "gcs": {
                    "demo": self.demo,
                    "enabled": self.enabled,
                    "throttle": self.throttle,
                    "steering": self.steering,
                    "client_command_age_ms": None if age is None else int(age * 1000),
                    "esp_port": self.esp_port,
                    "esp_baud": self.esp_baud,
                    "esp_error": self.last_esp_error,
                    "esp_connected": not self.demo and self.last_esp_error == "" and main_age is not None and main_age <= 2.5,
                    "main_status_age_ms": None if main_age is None else int(main_age * 1000),
                },
                "telemetry": self.telemetry,
                "sensors": self.sensors,
                "mission": self._mission_snapshot(),
                "mission_runtime": self.mission_controller.snapshot(),
                "test_log": self.test_logger.status(),
            }

    def update_mission(self, action: str, lat: Any = None, lon: Any = None, index: Any = None) -> dict[str, Any]:
        with self.lock:
            if action == "add":
                try:
                    latitude = float(lat)
                    longitude = float(lon)
                except (TypeError, ValueError) as exc:
                    raise ValueError("Latitude and longitude must be numbers.") from exc
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise ValueError("Latitude or longitude is outside its valid range.")
                if len(self.waypoints) >= 50:
                    raise ValueError("Mission is limited to 50 waypoints.")
                self.waypoints.append({"lat": round(latitude, 7), "lon": round(longitude, 7)})
            elif action == "remove":
                try:
                    waypoint_index = int(index)
                except (TypeError, ValueError) as exc:
                    raise ValueError("Waypoint index is invalid.") from exc
                if waypoint_index < 0 or waypoint_index >= len(self.waypoints):
                    raise ValueError("Waypoint index is outside the mission.")
                self.waypoints.pop(waypoint_index)
            elif action == "clear":
                self.waypoints.clear()
            elif action == "apply":
                if self.mission_controller.status == "RUNNING":
                    self.mission_controller.pause("Mission changed; apply again and restart to run the new route.")
                self.active_waypoints = list(self.waypoints)
                self.mission_revision += 1
                self._save_mission()
            elif action == "revert":
                self.waypoints = list(self.active_waypoints)
            else:
                raise ValueError("Unsupported mission action.")
            return self._mission_snapshot()

    def control_mission(self, action: str) -> dict[str, Any]:
        with self.lock:
            if action == "start":
                if self.enabled:
                    raise ValueError("Disable Manual Output before starting a mission.")
                if not self.demo:
                    main = self.telemetry.get("main", {})
                    ready = main.get("mode") == "AUTO" and main.get("rc_mode") == "AUTO" and main.get("rc_ok") is True
                    if not ready:
                        raise ValueError("Main must report RC-valid AUTO mode before mission start.")
                gps = self.sensors.get("gps", {})
                start_position = None
                if isinstance(gps.get("lat"), (int, float)) and isinstance(gps.get("lon"), (int, float)):
                    start_position = {"lat": float(gps["lat"]), "lon": float(gps["lon"])}
                self.mission_controller.start(self.active_waypoints, start_position)
                if not self.test_logger.active:
                    self.test_logger.start({"trigger": "mission_start", "waypoints": self.active_waypoints})
            elif action == "pause":
                self.mission_controller.pause()
            elif action == "stop":
                self.mission_controller.stop()
            else:
                raise ValueError("Unsupported mission control action.")
            return self.mission_controller.snapshot()

    def control_test_log(self, action: str) -> dict[str, Any]:
        with self.lock:
            if action == "start":
                self.test_logger.start({"trigger": "operator_start", "waypoints": self.active_waypoints})
            elif action == "stop":
                self.test_logger.stop("Stopped by operator.")
            else:
                raise ValueError("Unsupported test log action.")
            return self.test_logger.status()

    def set_mission_tuning(self, cruise_throttle: Any) -> dict[str, Any]:
        try:
            value = float(cruise_throttle)
        except (TypeError, ValueError) as exc:
            raise ValueError("Cruise throttle must be a number.") from exc
        with self.lock:
            self.mission_controller.set_cruise_throttle(value)
            return self.mission_controller.snapshot()

    def test_log_path(self, kind: str) -> str | None:
        path = self.test_logger.path_for(kind)
        return str(path) if path else None

    def _test_log_sample_locked(self) -> dict[str, Any]:
        main = self.telemetry.get("main", {})
        left = self.telemetry.get("left", {})
        right = self.telemetry.get("right", {})
        gps = self.sensors.get("gps", {})
        mag = self.sensors.get("mag", {})
        runtime = self.mission_controller.snapshot()
        target = runtime.get("target") or {}
        return {
            "mission_status": runtime.get("status"),
            "navigation_state": runtime.get("navigation_state"),
            "waypoint_index": runtime.get("waypoint_index"),
            "waypoint_total": runtime.get("waypoint_total"),
            "target_lat": target.get("lat"), "target_lon": target.get("lon"),
            "gps_lat": gps.get("lat"), "gps_lon": gps.get("lon"),
            "gps_sog_mps": gps.get("speed_mps"), "gps_hdop": gps.get("hdop"),
            "gps_satellites": gps.get("satellites"), "gps_fix_quality": gps.get("fix_quality"),
            "waypoint_distance_m": runtime.get("distance_m"),
            "cross_track_error_m": runtime.get("cross_track_error_m"),
            "target_bearing_deg": runtime.get("bearing_deg"),
            "actual_heading_deg": mag.get("heading_deg"), "actual_heading_raw_deg": mag.get("heading_raw_deg"),
            "heading_error_deg": runtime.get("heading_error_deg"), "yaw_rate_dps": runtime.get("yaw_rate_dps"),
            "cruise_throttle_setting": runtime.get("cruise_throttle"),
            "command_throttle": self.last_sent_throttle, "command_steering": self.last_sent_steering,
            "left_pwm_us": main.get("left_pwm_us"), "right_pwm_us": main.get("right_pwm_us"),
            "left_actual_pwm_us": left.get("actual_pwm_us"), "right_actual_pwm_us": right.get("actual_pwm_us"),
            "rc_ok": main.get("rc_ok"), "rc_mode": main.get("rc_mode"),
            "control_mode": main.get("mode"), "control_source": main.get("control_source"),
            "manual_latch": main.get("manual_latch"),
            "can_ok": main.get("can_ok"), "can_state": main.get("can_state"),
            "can_peers_fresh": main.get("can_peers_fresh"), "can_rearm_required": main.get("can_rearm_required"),
            "can_recoveries": main.get("can_recoveries"),
            "can_tx_error_counter": main.get("can_tx_error_counter"),
            "can_rx_error_counter": main.get("can_rx_error_counter"),
            "can_tx_failed_count": main.get("can_tx_failed_count"),
            "can_queue_failures": main.get("can_queue_failures"),
            "can_stall_restarts": main.get("can_stall_restarts"),
            "can_last_tx_error": main.get("can_last_tx_error"),
            "can_last_driver_error": main.get("can_last_driver_error"),
            "left_status_age_ms": main.get("left_status_age_ms"), "right_status_age_ms": main.get("right_status_age_ms"),
            "left_fault_flags": left.get("fault_flags"), "right_fault_flags": right.get("fault_flags"),
            "main_vbat_v": main.get("vbat_v"), "main_battery_valid": main.get("battery_valid"),
            "left_vbat_v": left.get("vbat_v"), "right_vbat_v": right.get("vbat_v"),
        }

    def run_test_logger_loop(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                sample = self._test_log_sample_locked()
                events = self.mission_controller.consume_events()
                main = self.telemetry.get("main", {})
                rc_state = tuple(main.get(key) for key in ("rc_ok", "rc_mode", "mode", "control_source", "manual_latch"))
                can_state = tuple(main.get(key) for key in ("can_ok", "can_state", "can_peers_fresh", "can_rearm_required"))
                if self._last_rc_state is not None and rc_state != self._last_rc_state:
                    events.append({"type": "rc_control_state_change", "previous": self._last_rc_state, "current": rc_state})
                if self._last_can_state is not None and can_state != self._last_can_state:
                    events.append({"type": "can_state_change", "previous": self._last_can_state, "current": can_state})
                self._last_rc_state = rc_state
                self._last_can_state = can_state
                active = self.test_logger.active
            if active:
                self.test_logger.record_sample(sample)
                for event in events:
                    self.test_logger.record_event(event, sample)
            self.stop_event.wait(0.2)

    def _mission_snapshot(self) -> dict[str, Any]:
        return {
            "waypoints": list(self.waypoints),
            "active_waypoints": list(self.active_waypoints),
            "pending_changes": self.waypoints != self.active_waypoints,
            "revision": self.mission_revision,
            "storage": "memory" if self.mission_file is None else self.mission_file,
            "execution": "local_pi",
        }

    def _load_mission(self) -> tuple[list[dict[str, float]], int]:
        if self.mission_file is None or not os.path.isfile(self.mission_file):
            return [], 0
        try:
            with open(self.mission_file, "r", encoding="utf-8") as file:
                data = json.load(file)
            waypoints = data.get("waypoints", [])
            if not isinstance(waypoints, list) or len(waypoints) > 50:
                raise ValueError("Mission file has invalid waypoints.")
            normalized = []
            for waypoint in waypoints:
                latitude = float(waypoint["lat"])
                longitude = float(waypoint["lon"])
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise ValueError("Mission file has invalid coordinates.")
                normalized.append({"lat": round(latitude, 7), "lon": round(longitude, 7)})
            return normalized, int(data.get("revision", 0))
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.last_esp_error = f"Mission file ignored: {exc}"
            return [], 0

    def _save_mission(self) -> None:
        if self.mission_file is None:
            return
        directory = os.path.dirname(os.path.abspath(self.mission_file))
        os.makedirs(directory, exist_ok=True)
        temporary = f"{self.mission_file}.tmp"
        payload = {"revision": self.mission_revision, "waypoints": self.active_waypoints}
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(payload, file, indent=2)
            file.write("\n")
        os.replace(temporary, self.mission_file)

    def run_esp_loop(self) -> None:
        from esp_link_v2 import EspMainLink
        esp = None
        next_command = time.time()

        while not self.stop_event.is_set():
            if esp is None:
                try:
                    esp = EspMainLink(self.esp_port, self.esp_baud, timeout=0.01)
                    with self.lock:
                        self.last_esp_error = ""
                        self.last_esp_connect = time.time()
                except Exception as exc:
                    with self.lock:
                        self.last_esp_error = str(exc)
                    time.sleep(1.0)
                    continue

            now = time.time()
            monotonic_now = time.monotonic()
            try:
                telemetry = esp.poll()
                with self.lock:
                    self.telemetry = {
                        "main": telemetry.main_status,
                        "left": telemetry.left_sub_status,
                        "right": telemetry.right_sub_status,
                        "raw": telemetry.raw_lines[-20:],
                    }
                    if esp.last_main_status_at is not None:
                        self.last_esp_status = esp.last_main_status_at

                # Main first waits for USB serial and can wait up to 5 seconds
                # for Wi-Fi/OTA startup. Beyond this window, reopen the host
                # USB endpoint instead of presenting a false connected state.
                if monotonic_now - esp.opened_at > 10.0 and esp.last_main_status_at is None:
                    raise TimeoutError("Main ESP sent no main_status within 10 seconds; reopening serial link")
                if esp.last_main_status_at is not None and monotonic_now - esp.last_main_status_at > 3.0:
                    raise TimeoutError("Main ESP main_status timed out; reopening serial link")

                if now >= next_command:
                    next_command = now + self.command_period
                    with self.lock:
                        client_fresh = self.last_client_command and now - self.last_client_command <= 0.7
                        if self.mission_controller.status == "RUNNING":
                            throttle, steering = self.mission_controller.tick(
                                self.sensors.get("gps", {}), self.sensors.get("mag", {}), self.sensors.get("imu", {})
                            )
                        else:
                            enabled = self.enabled and client_fresh
                            throttle = self.throttle if enabled else 0.0
                            steering = self.steering if enabled else 0.0

                    esp.send_manual_command(throttle, steering)
                    with self.lock:
                        self.last_sent_throttle = throttle
                        self.last_sent_steering = steering

            except Exception as exc:
                with self.lock:
                    self.last_esp_error = str(exc)
                    self.enabled = False
                    self.throttle = 0.0
                    self.steering = 0.0
                with suppress(Exception):
                    esp.close()
                esp = None
                time.sleep(0.5)

            time.sleep(0.005)

        if esp is not None:
            with suppress(Exception):
                for _ in range(5):
                    esp.send_neutral()
                    time.sleep(0.05)
                esp.close()

    def run_sensor_loop(self) -> None:
        from sensors_v2 import SensorManager
        manager = SensorManager(self.sensor_config)

        while not self.stop_event.is_set():
            snapshot = manager.read()
            with self.lock:
                self.sensors = snapshot
            time.sleep(0.1)

    def run_demo_loop(self) -> None:
        from demo_data import make_demo_data

        started = time.monotonic()
        while not self.stop_event.is_set():
            with self.lock:
                if self.mission_controller.status == "RUNNING":
                    self.mission_controller.tick(
                        self.sensors.get("gps", {}), self.sensors.get("mag", {}), self.sensors.get("imu", {})
                    )
                fresh = bool(self.last_client_command and time.time() - self.last_client_command <= 0.7)
                active = self.enabled and fresh
                self.telemetry, self.sensors = make_demo_data(
                    time.monotonic() - started,
                    self.throttle if active else 0.0,
                    self.steering if active else 0.0,
                    active,
                )
                self.last_sent_throttle = self.throttle if active else 0.0
                self.last_sent_steering = self.steering if active else 0.0
            self.stop_event.wait(0.1)

    @staticmethod
    def _clamp_unit(value: float) -> float:
        if value < -1.0:
            return -1.0
        if value > 1.0:
            return 1.0
        return value


def make_handler(state: GcsState):
    class GcsHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/api/status":
                self._send_json(state.snapshot())
                return

            if parsed.path == "/api/test-log/download":
                kind = parse_qs(parsed.query).get("kind", ["csv"])[0]
                file_path = state.test_log_path(kind)
                if not file_path or not os.path.isfile(file_path):
                    self.send_error(404, "Test log file not available")
                    return
                with open(file_path, "rb") as file:
                    body = file.read()
                content_type = "text/csv; charset=utf-8" if kind == "csv" else "application/json; charset=utf-8"
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(file_path)}"')
                self.end_headers()
                self.wfile.write(body)
                return

            path = parsed.path
            if path == "/":
                path = "/index.html"
            file_path = os.path.abspath(os.path.join(STATIC_DIR, path.lstrip("/")))
            static_root = os.path.abspath(STATIC_DIR)

            if not file_path.startswith(static_root) or not os.path.isfile(file_path):
                self.send_error(404)
                return

            content_type = mimetypes.guess_type(file_path)[0] or "application/octet-stream"
            with open(file_path, "rb") as file:
                body = file.read()

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            # GCS files are updated directly on the Pi during field work.
            # Avoid a browser retaining an older app.js after a restart.
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length).decode("utf-8") if length else ""

            if parsed.path == "/api/command":
                payload = json.loads(body or "{}")
                state.set_command(
                    float(payload.get("throttle", 0.0)),
                    float(payload.get("steering", 0.0)),
                    bool(payload.get("enabled", False)),
                )
                self._send_json({"ok": True})
                return

            if parsed.path == "/api/stop":
                state.stop_output()
                self._send_json({"ok": True})
                return

            if parsed.path == "/api/mission":
                try:
                    payload = json.loads(body or "{}")
                    mission = state.update_mission(
                        str(payload.get("action", "")),
                        payload.get("lat"),
                        payload.get("lon"),
                        payload.get("index"),
                    )
                except (json.JSONDecodeError, ValueError) as exc:
                    self._send_json({"ok": False, "error": str(exc)}, status=400)
                    return
                self._send_json({"ok": True, "mission": mission})
                return

            if parsed.path == "/api/mission/control":
                try:
                    payload = json.loads(body or "{}")
                    runtime = state.control_mission(str(payload.get("action", "")))
                except (json.JSONDecodeError, ValueError) as exc:
                    self._send_json({"ok": False, "error": str(exc)}, status=400)
                    return
                self._send_json({"ok": True, "mission_runtime": runtime})
                return

            if parsed.path == "/api/test-log":
                try:
                    payload = json.loads(body or "{}")
                    test_log = state.control_test_log(str(payload.get("action", "")))
                except (json.JSONDecodeError, ValueError) as exc:
                    self._send_json({"ok": False, "error": str(exc)}, status=400)
                    return
                self._send_json({"ok": True, "test_log": test_log})
                return

            if parsed.path == "/api/mission/tuning":
                try:
                    payload = json.loads(body or "{}")
                    runtime = state.set_mission_tuning(payload.get("cruise_throttle"))
                except (json.JSONDecodeError, ValueError) as exc:
                    self._send_json({"ok": False, "error": str(exc)}, status=400)
                    return
                self._send_json({"ok": True, "mission_runtime": runtime})
                return

            if parsed.path == "/api/query-command":
                query = parse_qs(parsed.query)
                state.set_command(
                    float(query.get("throttle", ["0"])[0]),
                    float(query.get("steering", ["0"])[0]),
                    query.get("enabled", ["false"])[0].lower() == "true",
                )
                self._send_json({"ok": True})
                return

            self.send_error(404)

        def log_message(self, format: str, *args: Any) -> None:
            return

        def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return GcsHandler


def main() -> None:
    parser = argparse.ArgumentParser(description="BN-USV V2 local GCS server")
    parser.add_argument("--demo", action="store_true", help="Preview with simulated data; no hardware connections or commands")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    # The installed USV Raspberry Pi enumerates the Main ESP USB link here.
    # Use --esp-port to select a different device if enumeration changes.
    parser.add_argument("--esp-port", default="/dev/ttyACM0")
    parser.add_argument("--esp-baud", type=int, default=115200)
    parser.add_argument("--command-rate", type=float, default=20.0)
    parser.add_argument("--gps-port", default="/dev/ttyAMA0")
    parser.add_argument("--gps-baud", type=int, default=9600)
    parser.add_argument("--i2c-bus", type=int, default=1)
    parser.add_argument("--imu-addr", default="0x68")
    parser.add_argument("--mag-addr", default="0x1c")
    parser.add_argument("--heading-median-window", type=int, default=5)
    parser.add_argument("--heading-offset-deg", type=float, default=0.0,
                        help="Compass installation offset applied before heading filtering")
    parser.add_argument("--heading-invert", action=argparse.BooleanOptionalAction, default=False,
                        help="Invert compass rotation to match the vessel/GCS heading convention")
    parser.add_argument("--ina226-addr", default="0x40")
    parser.add_argument("--mission-file", default=DEFAULT_MISSION_FILE,
                        help="Applied mission JSON file; use an empty value to keep mission in memory")
    args = parser.parse_args()

    if args.command_rate <= 0:
        parser.error("--command-rate must be positive")
    if args.heading_median_window < 1:
        parser.error("--heading-median-window must be positive")
    sensor_config = None
    if not args.demo:
        from sensors_v2 import SensorConfig

        sensor_config = SensorConfig(
            gps_port=args.gps_port,
            gps_baud=args.gps_baud,
            i2c_bus=args.i2c_bus,
            imu_addr=int(args.imu_addr, 0),
            mag_addr=int(args.mag_addr, 0),
            mag_heading_median_window=args.heading_median_window,
            mag_heading_offset_deg=args.heading_offset_deg,
            mag_heading_invert=args.heading_invert,
            ina226_addr=int(args.ina226_addr, 0),
        )

    mission_file = None if args.demo or not args.mission_file else args.mission_file
    state = GcsState("DEMO" if args.demo else args.esp_port, args.esp_baud, args.command_rate,
                     sensor_config, demo=args.demo, mission_file=mission_file)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    targets = [state.run_demo_loop, state.run_test_logger_loop] if args.demo else [state.run_esp_loop, state.run_sensor_loop, state.run_test_logger_loop]
    workers = [threading.Thread(target=target, daemon=True) for target in targets]
    for worker in workers:
        worker.start()
    print(f"BN-USV GCS running at http://{args.host}:{args.port}")
    print("DEMO: simulated data only; hardware disconnected" if args.demo else f"Main ESP serial: {args.esp_port} @ {args.esp_baud}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping GCS...")
    finally:
        state.stop_output()
        state.test_logger.stop("GCS shutting down.")
        state.stop_event.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=2.0)
        print("GCS stopped.")


if __name__ == "__main__":
    main()
