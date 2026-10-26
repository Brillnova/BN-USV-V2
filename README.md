# BN-USV V2 — Field Test Prototype

BN-USV V2 is a modular, 3D-printed unmanned surface vessel prototype. This folder contains the Raspberry Pi ground-control application and ESP32 controller firmware used in its development, shared as a public development snapshot.

**Prototype status:** RC control and Main/Sub communication were exercised during field testing. Waypoint navigation experienced unstable heading acquisition and has not been validated for reliable autonomous operation. Subsequent control changes require further bench and water testing. This is a research prototype, not a finished or certified product.

## Contents

- `software/pi_v2/`: local GCS, sensors, mission controller and simulated demo.
- `software/firmware/`: Main ESP32-S3 and Left/Right ESP32-C3 Sub sketches, shared CAN code and tests.
- `hardware/stl/`: prototype hull and accessory STL files, organized by part.
- `docs/`: project page, system overview, limitations and sharing checklist.

## Preview the GCS

```sh
cd software/pi_v2
python gcs_server.py --demo --host 127.0.0.1 --port 8000
```

Open <http://127.0.0.1:8000>. The demo uses simulated telemetry; map tiles require internet access. For installation and hardware setup see [Pi setup](software/pi_v2/README.md) and the [firmware notes](software/firmware/README.md).

Sensor defaults in this public copy are **uncalibrated examples**, not settings for any particular boat. Verify wiring, battery dividers, sensor axes, heading and failsafes before any hardware test. Secure the vessel and remove propellers during bench testing.

The source CAD, manufacturing/PCB files, field logs, real mission coordinates, local configuration and credentials are intentionally excluded. The STL models are prototypes and may require supports or further changes for reliable printing and sealing. See [sharing checklist](docs/PUBLICATION_CHECKLIST.md). 

## License

Brillnova-authored software and firmware in `software/` are licensed under the [MIT License](software/LICENSE). Vendored Leaflet is third-party software and retains its [own license](software/pi_v2/gcs_static/vendor/LEAFLET-LICENSE.txt).

Brillnova STL models in [`hardware/stl/`](hardware/stl/README.md) are licensed separately under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/). Commercial use requires separate permission. The MIT software license does not apply to the STL models, branding, photographs or other non-software assets.

## Project page

Preview the [combined V2 and V1 project page](docs/index.html) locally. See [Pages setup](docs/PAGES_SETUP.md) for publishing after the repository is created.
