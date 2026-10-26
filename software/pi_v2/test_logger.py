"""Durable flight-test logging for BN-USV V2.

Samples are written as CSV for spreadsheet review. Mission/RC/CAN events share
the same file and a compact JSON summary is produced when a session ends.
"""

from __future__ import annotations

import csv
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


CSV_FIELDS = [
    "timestamp_utc", "timestamp_unix", "record_type", "event", "event_detail_json",
    "mission_status", "navigation_state", "waypoint_index", "waypoint_total", "target_lat", "target_lon",
    "gps_lat", "gps_lon", "gps_sog_mps", "gps_hdop", "gps_satellites", "gps_fix_quality",
    "waypoint_distance_m", "cross_track_error_m", "target_bearing_deg", "actual_heading_deg", "actual_heading_raw_deg",
    "heading_error_deg", "yaw_rate_dps", "cruise_throttle_setting", "command_throttle", "command_steering",
    "left_pwm_us", "right_pwm_us", "left_actual_pwm_us", "right_actual_pwm_us",
    "rc_ok", "rc_mode", "control_mode", "control_source", "manual_latch",
    "can_ok", "can_state", "can_peers_fresh", "can_rearm_required",
    "can_recoveries", "can_tx_error_counter", "can_rx_error_counter", "can_tx_failed_count",
    "can_queue_failures", "can_stall_restarts", "can_last_tx_error", "can_last_driver_error",
    "left_status_age_ms", "right_status_age_ms", "left_fault_flags", "right_fault_flags",
    "main_vbat_v", "main_battery_valid", "left_vbat_v", "right_vbat_v",
]


class TestLogger:
    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory)
        self._lock = threading.Lock()
        self._file = None
        self._writer: csv.DictWriter | None = None
        self._csv_path: Path | None = None
        self._summary_path: Path | None = None
        self._started_at: datetime | None = None
        self._sample_count = 0
        self._events: list[dict[str, Any]] = []
        self._legs: dict[int, dict[str, float]] = {}

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self, metadata: dict[str, Any] | None = None) -> dict[str, Any]:
        with self._lock:
            if self.active:
                return self.status()
            self.directory.mkdir(parents=True, exist_ok=True)
            self._started_at = datetime.now(timezone.utc)
            stem = self._started_at.strftime("usv_test_%Y%m%d_%H%M%SZ")
            self._csv_path = self.directory / f"{stem}.csv"
            self._summary_path = self.directory / f"{stem}_summary.json"
            self._file = self._csv_path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS, extrasaction="ignore")
            self._writer.writeheader()
            self._sample_count = 0
            self._events = []
            self._legs = {}
            self._write("session_start", "session_start", {}, metadata or {})
            self._file.flush()
            return self.status()

    def record_sample(self, sample: dict[str, Any]) -> None:
        with self._lock:
            if not self.active:
                return
            self._write("sample", "", sample, {})
            self._sample_count += 1
            self._update_leg_stats(sample)
            if self._sample_count % 5 == 0:
                self._file.flush()

    def record_event(self, event: dict[str, Any], sample: dict[str, Any]) -> None:
        with self._lock:
            if not self.active:
                return
            name = str(event.get("type", "event"))
            detail = dict(event)
            self._events.append(detail)
            self._write("event", name, sample, detail)
            self._file.flush()

    def stop(self, reason: str = "Stopped by operator.") -> dict[str, Any]:
        with self._lock:
            if not self.active:
                return self.status()
            self._write("session_stop", "session_stop", {}, {"reason": reason})
            self._file.flush()
            self._file.close()
            completed = self.status(active=False)
            summary = {
                "started_at_utc": self._started_at.isoformat() if self._started_at else None,
                "ended_at_utc": datetime.now(timezone.utc).isoformat(),
                "reason": reason,
                "sample_count": self._sample_count,
                "csv_file": completed.get("csv_file"),
                "events": self._events,
                "legs": [
                    {"waypoint_index": index, **stats,
                     "average_sog_mps": stats["sum_sog_mps"] / stats["sample_count"] if stats["sample_count"] else None}
                    for index, stats in sorted(self._legs.items())
                ],
            }
            assert self._summary_path is not None
            self._summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
            self._file = None
            self._writer = None
            return completed

    def status(self, active: bool | None = None) -> dict[str, Any]:
        is_active = self.active if active is None else active
        return {
            "active": is_active,
            "csv_file": self._csv_path.name if self._csv_path else None,
            "summary_file": self._summary_path.name if self._summary_path else None,
            "sample_count": self._sample_count,
            "directory": str(self.directory),
        }

    def path_for(self, kind: str) -> Path | None:
        with self._lock:
            return self._csv_path if kind == "csv" else self._summary_path if kind == "summary" else None

    def _write(self, record_type: str, event: str, sample: dict[str, Any], detail: dict[str, Any]) -> None:
        assert self._writer is not None
        now = datetime.now(timezone.utc)
        row = {field: sample.get(field) for field in CSV_FIELDS}
        row.update({
            "timestamp_utc": now.isoformat(),
            "timestamp_unix": now.timestamp(),
            "record_type": record_type,
            "event": event,
            "event_detail_json": json.dumps(detail, separators=(",", ":")) if detail else "",
        })
        self._writer.writerow(row)

    def _update_leg_stats(self, sample: dict[str, Any]) -> None:
        if sample.get("mission_status") != "RUNNING":
            return
        index = sample.get("waypoint_index")
        speed = sample.get("gps_sog_mps")
        if not isinstance(index, int) or not isinstance(speed, (int, float)):
            return
        stats = self._legs.setdefault(index, {"sample_count": 0, "sum_sog_mps": 0.0, "max_sog_mps": 0.0})
        stats["sample_count"] += 1
        stats["sum_sog_mps"] += float(speed)
        stats["max_sog_mps"] = max(stats["max_sog_mps"], float(speed))
