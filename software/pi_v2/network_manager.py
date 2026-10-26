#!/usr/bin/env python3
"""Boot-time network selection for the BN-USV Raspberry Pi.

The manager first tries saved NetworkManager Wi-Fi profiles, including a saved
phone-tethering profile. When none can be connected, it starts the Pi's own
password-protected Wi-Fi access point so the local GCS remains reachable.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


CONFIG_PATH = Path(__file__).with_name("network_config.json")
DEFAULT_CONFIG: dict[str, Any] = {
    "wifi_interface": "wlan0",
    "preferred_profiles": [],
    "connect_timeout_sec": 20,
    "ap_ssid": "BN-USV-GCS",
    "ap_password": "CHANGE_ME",
    "ap_ipv4_address": "10.42.0.1/24",
}


def run(*args: str, check: bool = False) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(args, text=True, capture_output=True)
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)
    if check and result.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(args)}")
    return result


def load_config() -> dict[str, Any]:
    config = dict(DEFAULT_CONFIG)
    if not CONFIG_PATH.exists():
        print(f"Configuration file not found: {CONFIG_PATH}", file=sys.stderr)
        print("Copy network_config.example.json to network_config.json and set the AP password.", file=sys.stderr)
        return config

    try:
        supplied = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Cannot read {CONFIG_PATH}: {exc}") from exc

    if not isinstance(supplied, dict):
        raise RuntimeError("Network configuration must be a JSON object.")
    config.update(supplied)
    return config


def active_wifi_profile(interface: str) -> str | None:
    result = run("nmcli", "-t", "-f", "NAME,DEVICE,TYPE", "connection", "show", "--active")
    for line in result.stdout.splitlines():
        parts = line.split(":", 2)
        if len(parts) == 3 and parts[1] == interface and parts[2] in ("wifi", "802-11-wireless"):
            return parts[0]
    return None


def saved_wifi_profiles(excluded: set[str]) -> list[str]:
    result = run("nmcli", "-t", "-f", "NAME,TYPE", "connection", "show")
    profiles: list[str] = []
    for line in result.stdout.splitlines():
        parts = line.split(":", 1)
        if len(parts) == 2 and parts[1] in ("wifi", "802-11-wireless") and parts[0] not in excluded:
            profiles.append(parts[0])
    return profiles


def stop_access_point(ap_ssid: str) -> None:
    for connection in ("Hotspot", ap_ssid):
        run("nmcli", "connection", "down", "id", connection)


def connect_saved_wifi(config: dict[str, Any]) -> bool:
    interface = str(config["wifi_interface"])
    ap_ssid = str(config["ap_ssid"])
    run("nmcli", "radio", "wifi", "on")
    stop_access_point(ap_ssid)

    excluded = {"Hotspot", ap_ssid}
    preferred = [str(profile) for profile in config.get("preferred_profiles", [])]
    candidates = preferred + [profile for profile in saved_wifi_profiles(excluded) if profile not in preferred]

    already_active = active_wifi_profile(interface)
    # When priorities are configured, attempt the first preferred profile even
    # if NetworkManager auto-connected a lower-priority saved network first.
    if already_active and (not candidates or already_active == candidates[0]):
        print(f"Wi-Fi already connected: {already_active}")
        return True

    for profile in candidates:
        if profile == already_active:
            print(f"Wi-Fi already connected: {profile}")
            return True
        print(f"Trying saved Wi-Fi profile: {profile}")
        result = run("nmcli", "connection", "up", "id", profile, "ifname", interface)
        if result.returncode == 0:
            print(f"Connected to Wi-Fi profile: {profile}")
            return True
    return False


def start_access_point(config: dict[str, Any]) -> None:
    interface = str(config["wifi_interface"])
    ap_ssid = str(config["ap_ssid"])
    ap_password = str(config["ap_password"])
    ap_ipv4_address = str(config["ap_ipv4_address"])
    if ap_password == "CHANGE_ME" or not 8 <= len(ap_password) <= 63:
        raise RuntimeError("Set ap_password in network_config.json to 8-63 characters before enabling AP fallback.")

    run("nmcli", "radio", "wifi", "on")
    stop_access_point(ap_ssid)
    result = run(
        "nmcli", "device", "wifi", "hotspot", "ifname", interface,
        "ssid", ap_ssid, "password", ap_password,
    )
    if result.returncode != 0:
        raise RuntimeError("NetworkManager could not start the fallback access point.")

    # NetworkManager's hotspot helper normally uses 10.42.0.1, but set it
    # explicitly so the field-test URL remains predictable.
    connection = active_wifi_profile(interface)
    if not connection:
        raise RuntimeError("Fallback access point started but has no active NetworkManager connection.")
    run(
        "nmcli", "connection", "modify", connection,
        "ipv4.method", "shared", "ipv4.addresses", ap_ipv4_address,
        "connection.autoconnect", "no",
        check=True,
    )
    run("nmcli", "connection", "down", "id", connection)
    run("nmcli", "connection", "up", "id", connection, "ifname", interface, check=True)
    print(f"Fallback AP started: {ap_ssid}")
    print(f"Connect a phone/tablet to this AP, then open http://{ap_ipv4_address.split('/', 1)[0]}:8000")


def auto_mode(config: dict[str, Any]) -> None:
    if connect_saved_wifi(config):
        return

    delay = max(0, int(config.get("connect_timeout_sec", 20)))
    if delay:
        print(f"No saved Wi-Fi connected; waiting {delay} seconds before AP fallback.")
        time.sleep(delay)
        if active_wifi_profile(str(config["wifi_interface"])):
            return
    start_access_point(config)


def main() -> None:
    parser = argparse.ArgumentParser(description="BN-USV Pi network mode manager")
    parser.add_argument("mode", choices=("auto", "wifi", "ap", "status"), nargs="?", default="auto")
    args = parser.parse_args()
    config = load_config()

    try:
        if args.mode == "auto":
            auto_mode(config)
        elif args.mode == "wifi":
            if not connect_saved_wifi(config):
                raise RuntimeError("No saved Wi-Fi profile could be connected.")
        elif args.mode == "ap":
            start_access_point(config)
        else:
            run("nmcli", "device", "status")
            run("nmcli", "connection", "show", "--active")
    except RuntimeError as exc:
        print(f"Network manager error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
