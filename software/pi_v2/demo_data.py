"""Synthetic display data only, not a navigation or vessel dynamics simulator."""
import json
import math


def make_demo_data(elapsed: float, throttle: float = 0.0, steering: float = 0.0, enabled: bool = False):
    heading = (85 + elapsed * 2) % 360
    roll = 7 * math.sin(elapsed * 0.5)
    pitch = 3 * math.sin(elapsed * 0.7)
    sub_voltage = 15.7 + 0.05 * math.sin(elapsed * 0.2)
    main_voltage = 7.85 + 0.03 * math.sin(elapsed * 0.2)
    # Controls affect only virtual PWM. Instrument motion is an independent preview.
    if not enabled:
        throttle = steering = 0.0
    left = round(max(1000, min(2000, 1500 + 500 * (throttle + steering))))
    right = round(max(1000, min(2000, 1500 + 500 * (throttle - steering))))
    main = {
        "demo": True, "can_ok": True, "can_state": "RUNNING",
        "mode": "AUTO" if enabled else "KILL", "control_source": "DEMO",
        "pi_command_fresh": enabled, "rc_ok": True, "rc_mode": "AUTO" if enabled else "KILL",
        "manual_latch": False, "ch1_us": 1500, "ch2_us": 1500,
        "ch4_us": 1500 if enabled else 2000,
        "left_pwm_us": left, "right_pwm_us": right,
        "left_status_age_ms": 35, "right_status_age_ms": 42,
    }
    telemetry = {"main": main}
    for label, node, pwm in (("left", 1, left), ("right", 2, right)):
        telemetry[label] = {
            "demo": True, "source_node": node, "seq": int(elapsed * 5) % 65536,
            "actual_pwm_us": pwm, "vbat_v": round(sub_voltage - node * 0.02, 2),
            "status_flags": 111 if enabled else 106, "fault_flags": 0,
        }
    telemetry["raw"] = [json.dumps({"type": "demo_status", "timestamp": round(elapsed, 2), "data": main})]
    sensors = {
        "gps": {"ok": True, "lat": 37.5665, "lon": 126.9780,
                "speed_mps": 0.8 + 0.2 * math.sin(elapsed * 0.3), "course_deg": heading,
                "hdop": 0.8, "satellites": 14, "fix_quality": 1},
        "imu": {"ok": True, "roll_deg": roll, "pitch_deg": pitch,
                "ax_g": -math.sin(math.radians(pitch)), "ay_g": math.sin(math.radians(roll)),
                "az_g": math.cos(math.radians(roll)), "gx_dps": 3.5 * math.cos(elapsed * 0.5),
                "gy_dps": 2.1 * math.cos(elapsed * 0.7), "gz_dps": 2.0},
        "mag": {"ok": True, "heading_deg": heading},
        "battery": {"ok": True, "voltage_v": main_voltage, "cell_v": main_voltage / 2,
                    "current_a": 2.4, "soc_percent": 72 + math.sin(elapsed * 0.1)},
    }
    return telemetry, sensors
