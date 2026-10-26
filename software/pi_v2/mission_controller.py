"""Conservative waypoint controller for the V2 Pi GCS.

AUTO navigation starts with a low-output forward-only alignment sequence. It
never uses differential reverse thrust to turn the catamaran.
"""

from __future__ import annotations

import math
import time
from typing import Any


ARRIVAL_RADIUS_M = 3.0
ARRIVAL_CONFIRMATIONS = 3
MAX_HDOP = 2.0
CRUISE_THROTTLE = 0.25

# Normalized output maps to 500 us on each Main input channel. These are
# initial field-test limits: 0.10 steering is a 50 us differential component.
ALIGN_THROTTLE = 0.12
ALIGN_MAX_STEERING = 0.10
CRUISE_MAX_STEERING = 0.15
STEERING_GAIN_DEG = 90.0
# Keep the derivative term off until a water test confirms the IMU yaw sign.
# Yaw rate is already used to gate STABILIZE, which has no sign ambiguity.
YAW_RATE_DAMPING_GAIN = 0.0
IMU_YAW_RATE_TO_CLOCKWISE = -1.0

# Main has its own PWM slew limiter. Pi also limits commanded changes: 0.20
# normalized/s corresponds to 100 us/s. Emergency neutral remains immediate.
THROTTLE_RAMP_UP_PER_SEC = 0.05
STEERING_SLEW_PER_SEC = 0.20

ALIGN_ENTRY_ERROR_DEG = 10.0
STABILIZE_MAX_YAW_RATE_DPS = 3.0
STABILIZE_HOLD_SEC = 1.5
ALIGN_TIMEOUT_SEC = 25.0
CRUISE_REACQUIRE_ERROR_DEG = 30.0


def _number(value: Any) -> float | None:
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi, dlon = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlon / 2) ** 2
    return radius * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2, dlon = math.radians(lat1), math.radians(lat2), math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def angle_error_deg(target: float, actual: float) -> float:
    return (target - actual + 180.0) % 360.0 - 180.0


def cross_track_error_m(start: dict[str, float], end: dict[str, float], point: dict[str, float]) -> float:
    """Return absolute distance from point to the finite local path segment."""
    radius = 6_371_000.0
    reference_lat = math.radians(start["lat"])

    def xy(position: dict[str, float]) -> tuple[float, float]:
        return (
            math.radians(position["lon"] - start["lon"]) * radius * math.cos(reference_lat),
            math.radians(position["lat"] - start["lat"]) * radius,
        )

    end_x, end_y = xy(end)
    point_x, point_y = xy(point)
    length_squared = end_x * end_x + end_y * end_y
    if length_squared <= 1e-9:
        return math.hypot(point_x, point_y)
    fraction = max(0.0, min(1.0, (point_x * end_x + point_y * end_y) / length_squared))
    return math.hypot(point_x - fraction * end_x, point_y - fraction * end_y)


class MissionController:
    """Waypoint executor with ALIGN, STABILIZE, and CRUISE navigation states."""

    def __init__(self) -> None:
        self.status = "IDLE"
        self.navigation_state = "IDLE"
        self.reason = "No mission started."
        self.waypoints: list[dict[str, float]] = []
        self.index = 0
        self.bearing: float | None = None
        self.distance: float | None = None
        self.heading_error: float | None = None
        self.cross_track: float | None = None
        self.yaw_rate_dps: float | None = None
        self.leg_start: dict[str, float] | None = None
        self.throttle = 0.0
        self.steering = 0.0
        self.cruise_throttle = CRUISE_THROTTLE
        self.arrival_samples = 0
        self._align_started_at: float | None = None
        self._stable_since: float | None = None
        self._last_throttle_update: float | None = None
        self._last_steering_update: float | None = None
        self._events: list[dict[str, Any]] = []

    def start(self, waypoints: list[dict[str, float]], start_position: dict[str, float] | None = None) -> None:
        if not waypoints:
            raise ValueError("Apply at least one waypoint before starting the mission.")
        self.waypoints = list(waypoints)
        self.index = 0
        self.arrival_samples = 0
        self.leg_start = dict(start_position) if start_position else None
        self.cross_track = None
        self.status = "RUNNING"
        self._set_navigation_state("ALIGN", "Aligning to first waypoint at limited output.")
        self._align_started_at = time.monotonic()
        self._stable_since = None
        self._neutral()
        self._events.append({"type": "mission_start", "waypoint_index": 0, "leg_start": self.leg_start})

    def pause(self, reason: str = "Paused by operator.") -> None:
        if self.status == "RUNNING":
            self.status = "PAUSED"
            self._events.append({"type": "mission_pause", "reason": reason, "waypoint_index": self.index})
        self.navigation_state = "PAUSED"
        self.reason = reason
        self._neutral()

    def stop(self, reason: str = "Stopped by operator.") -> None:
        self.status = "IDLE"
        self.navigation_state = "IDLE"
        self.index = 0
        self.arrival_samples = 0
        self.reason = reason
        self._events.append({"type": "mission_stop", "reason": reason, "waypoint_index": self.index})
        self._neutral()

    def tick(self, gps: dict[str, Any], mag: dict[str, Any], imu: dict[str, Any] | None = None,
             now: float | None = None) -> tuple[float, float]:
        current_time = time.monotonic() if now is None else now
        if self.status != "RUNNING":
            self._neutral()
            return self.throttle, self.steering

        lat, lon = _number(gps.get("lat")), _number(gps.get("lon"))
        heading = _number(mag.get("heading_deg"))
        hdop = _number(gps.get("hdop"))
        yaw_raw = _number((imu or {}).get("gz_dps"))
        if gps.get("ok") is not True or _number(gps.get("fix_quality")) is None or _number(gps.get("fix_quality")) <= 0:
            self.pause("GPS fix lost; output neutral.")
            return self.throttle, self.steering
        if hdop is None or hdop > MAX_HDOP:
            self.pause("GPS HDOP is outside the mission limit; output neutral.")
            return self.throttle, self.steering
        if lat is None or lon is None or heading is None:
            self.pause("GPS position or compass heading unavailable; output neutral.")
            return self.throttle, self.steering
        if (imu or {}).get("ok") is not True or yaw_raw is None:
            self.pause("IMU yaw rate unavailable; output neutral.")
            return self.throttle, self.steering
        if self.index >= len(self.waypoints):
            self.status = "COMPLETE"
            self.navigation_state = "COMPLETE"
            self.reason = "All waypoints reached."
            self._neutral()
            return self.throttle, self.steering

        target = self.waypoints[self.index]
        if self.leg_start is None:
            self.leg_start = {"lat": lat, "lon": lon}
        self.distance = distance_m(lat, lon, target["lat"], target["lon"])
        self.bearing = bearing_deg(lat, lon, target["lat"], target["lon"])
        self.heading_error = angle_error_deg(self.bearing, heading)
        self.yaw_rate_dps = IMU_YAW_RATE_TO_CLOCKWISE * yaw_raw
        self.cross_track = cross_track_error_m(self.leg_start, target, {"lat": lat, "lon": lon})
        if self.distance <= ARRIVAL_RADIUS_M:
            self.arrival_samples += 1
        else:
            self.arrival_samples = 0
        if self.arrival_samples >= ARRIVAL_CONFIRMATIONS:
            self._arrive(lat, lon, heading, hdop, target, current_time)
            return self.throttle, self.steering

        if self.navigation_state == "ALIGN":
            return self._tick_align(current_time)
        if self.navigation_state == "STABILIZE":
            return self._tick_stabilize(current_time)
        if self.navigation_state == "CRUISE":
            return self._tick_cruise(current_time)
        self._begin_align(current_time, "Restoring safe waypoint alignment.")
        return self.throttle, self.steering

    def snapshot(self) -> dict[str, Any]:
        target = self.waypoints[self.index] if self.status == "RUNNING" and self.index < len(self.waypoints) else None
        return {
            "status": self.status, "navigation_state": self.navigation_state, "reason": self.reason,
            "waypoint_index": self.index, "waypoint_total": len(self.waypoints), "target": target,
            "distance_m": self.distance, "bearing_deg": self.bearing, "heading_error_deg": self.heading_error,
            "cross_track_error_m": self.cross_track, "yaw_rate_dps": self.yaw_rate_dps, "leg_start": self.leg_start,
            "throttle": self.throttle, "cruise_throttle": self.cruise_throttle,
            "throttle_ramp_up_per_sec": THROTTLE_RAMP_UP_PER_SEC, "steering": self.steering,
            "steering_slew_per_sec": STEERING_SLEW_PER_SEC, "align_throttle": ALIGN_THROTTLE,
            "align_max_steering": ALIGN_MAX_STEERING, "cruise_max_steering": CRUISE_MAX_STEERING,
            "arrival_radius_m": ARRIVAL_RADIUS_M, "arrival_confirmations": ARRIVAL_CONFIRMATIONS, "max_hdop": MAX_HDOP,
        }

    def consume_events(self) -> list[dict[str, Any]]:
        events = list(self._events)
        self._events.clear()
        return events

    def set_cruise_throttle(self, value: float) -> None:
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("Cruise throttle must be between 0.00 and 1.00.")
        self.cruise_throttle = value
        self._events.append({"type": "cruise_throttle_changed", "cruise_throttle": value})

    def _tick_align(self, now: float) -> tuple[float, float]:
        if self._align_started_at is None:
            self._align_started_at = now
        if now - self._align_started_at > ALIGN_TIMEOUT_SEC:
            self.pause("Heading alignment timed out; output neutral.")
            return self.throttle, self.steering
        assert self.heading_error is not None and self.yaw_rate_dps is not None
        if abs(self.heading_error) <= ALIGN_ENTRY_ERROR_DEG:
            self._set_navigation_state("STABILIZE", "Heading aligned; waiting for turn rate to settle.")
            self._stable_since = None
            self._neutral()
            return self.throttle, self.steering
        self.reason = "Aligning to waypoint with forward-only limited differential thrust."
        self.throttle = self._ramp_throttle(ALIGN_THROTTLE, now)
        maximum = min(ALIGN_MAX_STEERING, self.throttle)
        self.steering = self._ramp_steering(self._turn_command(maximum), now)
        return self.throttle, self.steering

    def _tick_stabilize(self, now: float) -> tuple[float, float]:
        assert self.heading_error is not None and self.yaw_rate_dps is not None
        if abs(self.heading_error) > ALIGN_ENTRY_ERROR_DEG:
            self._begin_align(now, "Heading left alignment window; correcting again.")
            return self.throttle, self.steering
        if abs(self.yaw_rate_dps) > STABILIZE_MAX_YAW_RATE_DPS:
            self._stable_since = None
            self.reason = "Waiting for yaw rate to settle before cruise."
            self._neutral()
            return self.throttle, self.steering
        if self._stable_since is None:
            self._stable_since = now
        if now - self._stable_since >= STABILIZE_HOLD_SEC:
            self._set_navigation_state("CRUISE", "Heading stable; proceeding to waypoint.")
            self._neutral()
            return self.throttle, self.steering
        self.reason = "Heading stable; confirming before cruise."
        self._neutral()
        return self.throttle, self.steering

    def _tick_cruise(self, now: float) -> tuple[float, float]:
        assert self.heading_error is not None
        if abs(self.heading_error) > CRUISE_REACQUIRE_ERROR_DEG:
            self._begin_align(now, "Large heading error; returning to limited alignment.")
            return self.throttle, self.steering
        self.reason = "Cruising toward waypoint with limited heading correction."
        self.throttle = self._ramp_throttle(self.cruise_throttle, now)
        maximum = min(CRUISE_MAX_STEERING, self.throttle)
        self.steering = self._ramp_steering(self._turn_command(maximum), now)
        return self.throttle, self.steering

    def _arrive(self, lat: float, lon: float, heading: float, hdop: float,
                target: dict[str, float], now: float) -> None:
        arrived_index = self.index
        self._events.append({"type": "waypoint_arrival", "waypoint_index": arrived_index, "target": dict(target),
                             "arrival_lat": lat, "arrival_lon": lon, "arrival_error_m": self.distance,
                             "cross_track_error_m": self.cross_track, "hdop": hdop, "heading_deg": heading})
        self.index += 1
        self.arrival_samples = 0
        self._neutral()
        if self.index >= len(self.waypoints):
            self.status = "COMPLETE"
            self.navigation_state = "COMPLETE"
            self.reason = "All waypoints reached."
            self._events.append({"type": "mission_complete", "waypoint_index": arrived_index})
        else:
            self.leg_start = dict(target)
            self._begin_align(now, f"Waypoint {arrived_index + 1} reached; aligning to {self.index + 1}.")

    def _turn_command(self, maximum: float) -> float:
        assert self.heading_error is not None and self.yaw_rate_dps is not None
        value = self.heading_error / STEERING_GAIN_DEG - YAW_RATE_DAMPING_GAIN * self.yaw_rate_dps
        return max(-maximum, min(maximum, value))

    def _begin_align(self, now: float, reason: str) -> None:
        self._set_navigation_state("ALIGN", reason)
        self._align_started_at = now
        self._stable_since = None
        self._neutral()

    def _set_navigation_state(self, state: str, reason: str) -> None:
        if self.navigation_state != state:
            self._events.append({"type": "navigation_state", "state": state, "waypoint_index": self.index})
        self.navigation_state = state
        self.reason = reason

    def _neutral(self) -> None:
        self.throttle = 0.0
        self.steering = 0.0
        self._last_throttle_update = None
        self._last_steering_update = None

    def _ramp_throttle(self, target: float, now: float) -> float:
        if self._last_throttle_update is None:
            self._last_throttle_update = now
            return self.throttle
        dt = max(0.0, min(now - self._last_throttle_update, 0.5))
        self._last_throttle_update = now
        if target <= self.throttle:
            return target
        return min(target, self.throttle + THROTTLE_RAMP_UP_PER_SEC * dt)

    def _ramp_steering(self, target: float, now: float) -> float:
        if self._last_steering_update is None:
            self._last_steering_update = now
            return self.steering
        dt = max(0.0, min(now - self._last_steering_update, 0.5))
        self._last_steering_update = now
        step = STEERING_SLEW_PER_SEC * dt
        return max(self.steering - step, min(self.steering + step, target))
