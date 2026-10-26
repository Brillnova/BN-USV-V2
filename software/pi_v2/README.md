# Raspberry Pi ground control station

## Demo

```sh
python gcs_server.py --demo --host 127.0.0.1 --port 8000
```

Open http://127.0.0.1:8000. Demo mode uses simulated data and does not operate hardware. Map imagery requires network access.

## Raspberry Pi setup

Enable I2C and, if using GPIO UART for GPS, enable the serial interface and disable the serial login console. Install Python and dependencies:

```sh
cd software/pi_v2
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python gcs_server.py --esp-port /dev/ttyACM0 --gps-port /dev/ttyAMA0
```

Choose ports for your own Pi and wiring; `/dev/serial/by-id/` can help identify the ESP. Use `python gcs_server.py --help` for options. Access the GCS on port 8000 from a device on the same network. Restrict access to a trusted network: this prototype is not a hardened public-facing service.

## Calibration and field use

Copy `imu_calibration.example.json` to `imu_calibration.json` and configure axis signs and offsets for the installation. Magnetometer offsets/scales in `sensors_v2.py` are neutral placeholders; measure and set them for the actual unit. Verify heading direction and mounting offset using the command-line options. Neutral values do not constitute a calibration. Check GPS, battery sensing, RC authority and all failsafes before attempting autonomous operation. Field-tested RC does not mean waypoint autonomy is validated.

Mission plans and test logs are created locally at runtime and are excluded from this release. The network manager can use `network_config.example.json` as a starting point for a local `network_config.json`; configure saved NetworkManager profiles and a strong AP password for your own system. Boot services are not included in this candidate; configure startup only after checking the hardware and network behavior.

## Operating code and test utilities

`gcs_server.py` is the normal GCS entry point. Its imported modules, including `test_logger.py`, support normal operation: despite its name, `test_logger.py` records field-test sessions and is not a disposable unit test.

`main_prop_cmd_sender_test.py` is a separate diagnostic utility that sends Pi commands directly to Main and displays telemetry. It is not needed to start the GCS, and it may command propulsion when the control system permits it. Disconnect propulsion or remove propellers before using it. Avoid running it at the same time as the GCS on the same serial port.
