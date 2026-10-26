import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from battery import BatteryMonitor
from gps import GPS
from imu import IMU
from mag import Magnetometer


# This project's configured GPIO UART is ttyAMA0. On the current Pi, the
# serial0 alias targets ttyAMA10 while the verified GPS header UART is ttyAMA0.
GPS_PORT_DEFAULT = "/dev/ttyAMA0"
GPS_BAUD_DEFAULT = 9600
IMU_ADDR_DEFAULT = 0x68
MAG_ADDR_DEFAULT = 0x1C
INA226_ADDR_DEFAULT = 0x40
IMU_CALIBRATION_FILE = Path(__file__).with_name("imu_calibration.json")


def load_imu_calibration() -> tuple[bool, float, bool, float]:
    """Load display-axis sign and offsets without embedding field values in code."""
    defaults = (False, 0.0, False, 0.0)
    try:
        with IMU_CALIBRATION_FILE.open("r", encoding="utf-8") as file:
            calibration = json.load(file)
        roll = calibration["roll"]
        pitch = calibration["pitch"]
        return (
            bool(roll.get("invert", defaults[0])),
            float(roll.get("offset_deg", defaults[1])),
            bool(pitch.get("invert", defaults[2])),
            float(pitch.get("offset_deg", defaults[3])),
        )
    except (OSError, ValueError, KeyError, TypeError):
        return defaults


IMU_ROLL_INVERT, IMU_ROLL_OFFSET_DEG, IMU_PITCH_INVERT, IMU_PITCH_OFFSET_DEG = load_imu_calibration()


@dataclass
class SensorConfig:
    gps_port: str = GPS_PORT_DEFAULT
    gps_baud: int = GPS_BAUD_DEFAULT
    i2c_bus: int = 1
    imu_addr: int = IMU_ADDR_DEFAULT
    mag_addr: int = MAG_ADDR_DEFAULT
    ina226_addr: int = INA226_ADDR_DEFAULT
    mag_offset_x: float = 0.0
    mag_offset_y: float = 0.0
    mag_scale_x: float = 1.0
    mag_scale_y: float = 1.0
    mag_avg_scale: float = 1.0
    mag_heading_median_window: int = 5
    # Configure and calibrate for the actual sensor mounting.
    mag_heading_offset_deg: float = 0.0
    # Verify the direction against a known reference.
    mag_heading_invert: bool = False
    battery_capacity_mah: float = 10000.0
    battery_cell_count: int = 2
    battery_shunt_ohm: float = 0.002
    battery_current_lsb: float = 0.001
    imu_roll_invert: bool = IMU_ROLL_INVERT
    imu_roll_offset_deg: float = IMU_ROLL_OFFSET_DEG
    imu_pitch_invert: bool = IMU_PITCH_INVERT
    imu_pitch_offset_deg: float = IMU_PITCH_OFFSET_DEG


class GpsSensor:
    def __init__(self, config: SensorConfig):
        self.gps = GPS(config.gps_port, config.gps_baud)

    def read(self) -> dict[str, Any]:
        lat, lon, speed, course, hdop, satellites, fix_quality = self.gps.poll_position()
        return {
            "ok": lat is not None and lon is not None,
            "lat": lat,
            "lon": lon,
            "speed_mps": speed,
            "course_deg": course,
            "hdop": hdop,
            "satellites": satellites,
            "fix_quality": fix_quality,
        }


class ImuSensor:
    FILTER_ALPHA = 0.98
    MAX_DT_SEC = 0.2

    def __init__(self, config: SensorConfig):
        self.imu = IMU(bus_id=config.i2c_bus, address=config.imu_addr)
        self.imu.init()
        self.last_time = None
        self.roll_deg = None
        self.pitch_deg = None
        self.roll_sign = -1.0 if config.imu_roll_invert else 1.0
        self.pitch_sign = -1.0 if config.imu_pitch_invert else 1.0
        self.roll_offset_deg = config.imu_roll_offset_deg
        self.pitch_offset_deg = config.imu_pitch_offset_deg

    def read(self) -> dict[str, Any]:
        ax, ay, az = self.imu.read_accel()
        gx, gy, gz = self.imu.read_gyro()
        accel_roll_deg = self.roll_sign * math.degrees(math.atan2(ay, az))
        accel_pitch_deg = self.pitch_sign * math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        now = time.time()
        if self.last_time is None or self.roll_deg is None or self.pitch_deg is None:
            dt = 0.0
            self.roll_deg = accel_roll_deg
            self.pitch_deg = accel_pitch_deg
        else:
            dt = max(0.0, min(now - self.last_time, self.MAX_DT_SEC))
            gyro_roll_deg = self.roll_deg + self.roll_sign * gx * dt
            gyro_pitch_deg = self.pitch_deg + self.pitch_sign * gy * dt
            self.roll_deg = self.FILTER_ALPHA * gyro_roll_deg + (1.0 - self.FILTER_ALPHA) * accel_roll_deg
            self.pitch_deg = self.FILTER_ALPHA * gyro_pitch_deg + (1.0 - self.FILTER_ALPHA) * accel_pitch_deg

        self.last_time = now

        return {
            "ok": True,
            "ax_g": ax,
            "ay_g": ay,
            "az_g": az,
            "gx_dps": gx,
            "gy_dps": gy,
            "gz_dps": gz,
            "accel_roll_deg": accel_roll_deg + self.roll_offset_deg,
            "accel_pitch_deg": accel_pitch_deg + self.pitch_offset_deg,
            "roll_deg": self.roll_deg + self.roll_offset_deg,
            "pitch_deg": self.pitch_deg + self.pitch_offset_deg,
            "attitude_filter": "complementary",
            "filter_alpha": self.FILTER_ALPHA,
            "filter_dt_sec": dt,
            "roll_offset_deg": self.roll_offset_deg,
            "pitch_offset_deg": self.pitch_offset_deg,
        }


class MagnetometerSensor:
    def __init__(self, config: SensorConfig):
        self.mag = Magnetometer(
            address=config.mag_addr,
            offset_x=config.mag_offset_x,
            offset_y=config.mag_offset_y,
            scale_x=config.mag_scale_x,
            scale_y=config.mag_scale_y,
            avg_scale=config.mag_avg_scale,
            bus_num=config.i2c_bus,
        )
        self.mag.init()
        window = max(1, int(config.mag_heading_median_window))
        self.median_window = window if window % 2 else window + 1
        self.heading_offset_deg = float(config.mag_heading_offset_deg) % 360.0
        self.heading_invert = bool(config.mag_heading_invert)
        self.heading_samples: deque[float] = deque(maxlen=self.median_window)

    def read(self) -> dict[str, Any]:
        raw_heading = float(self.mag.get_heading()) % 360.0
        vessel_heading = ((-raw_heading if self.heading_invert else raw_heading) + self.heading_offset_deg) % 360.0
        self.heading_samples.append(vessel_heading)
        filtered_heading = circular_median_deg(self.heading_samples)
        return {
            "ok": True,
            # heading_deg is the filtered value used by GCS and autonomy.
            "heading_deg": filtered_heading,
            "heading_raw_deg": raw_heading,
            "heading_offset_deg": self.heading_offset_deg,
            "heading_inverted": self.heading_invert,
            "heading_filter": "circular_median",
            "heading_filter_window": self.median_window,
        }


def circular_median_deg(samples: Any) -> float:
    """Median-like robust heading estimate that remains correct across 0/360°."""
    values = [float(value) % 360.0 for value in samples]
    if not values:
        raise ValueError("At least one heading sample is required.")
    return min(
        values,
        key=lambda candidate: sum(abs((candidate - value + 180.0) % 360.0 - 180.0) for value in values),
    )


class BatterySensor:
    def __init__(self, config: SensorConfig):
        self.battery = BatteryMonitor(
            bus_id=config.i2c_bus,
            address=config.ina226_addr,
            shunt_ohm=config.battery_shunt_ohm,
            current_lsb=config.battery_current_lsb,
            capacity_mah=config.battery_capacity_mah,
            cell_count=config.battery_cell_count,
        )
        self.cell_count = config.battery_cell_count

    def read(self) -> dict[str, Any]:
        data = self.battery.read()
        voltage = data["voltage"]
        voltage_soc = data["voltage_soc"]
        return {
            "ok": True,
            "voltage_v": voltage,
            "current_a": data["current"],
            "cell_v": voltage / self.cell_count,
            "soc_percent": voltage_soc,
            "coulomb_soc_percent": data["coulomb_soc"],
            "used_mah": data["used_mah"],
            "remaining_mah": data["remaining_mah"],
            "initial_voltage_v": data["initial_voltage"],
            "initial_soc_percent": data["initial_soc"],
            "idle_correction_applied": data["idle_correction_applied"],
        }


class SensorManager:
    def __init__(self, config: SensorConfig):
        self.gps = self._make("gps", lambda: GpsSensor(config))
        self.imu = self._make("imu", lambda: ImuSensor(config))
        self.mag = self._make("mag", lambda: MagnetometerSensor(config))
        # Main 2S voltage comes from the Main ESP32-S3 ADC over USB telemetry.
        # Do not require an INA226 on the Pi I2C bus.
        self.battery = {"ok": True, "source": "Main ESP32-S3 ADC"}

    def read(self) -> dict[str, Any]:
        return {
            "gps": self._read_one(self.gps),
            "imu": self._read_one(self.imu),
            "mag": self._read_one(self.mag),
            "battery": self._read_one(self.battery),
        }

    @staticmethod
    def _make(name: str, factory):
        try:
            return factory()
        except Exception as exc:
            return {"ok": False, "name": name, "error": str(exc)}

    @staticmethod
    def _read_one(reader) -> dict[str, Any]:
        if isinstance(reader, dict):
            return reader
        try:
            return reader.read()
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
