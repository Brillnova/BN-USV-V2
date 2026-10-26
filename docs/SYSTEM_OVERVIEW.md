# System overview

The V2 prototype combines a central Main controller and two Sub controllers with a Raspberry Pi. The Pi serves a browser-based ground control station, reads navigation sensors, and exchanges commands and telemetry with Main over USB serial. Main receives RC input and communicates with the Left and Right Sub controllers over CAN. The Subs control the propulsion ESCs and report status and battery voltage.

The GCS provides telemetry, map-based mission editing, logging and a simulated demo. Map imagery is delivered separately from the locally bundled map engine and requires an available tile source. Connectivity and offline behavior must be tested in the intended environment.

Prototype hull and accessory STL files are included in `hardware/stl/`. Source CAD and PCB manufacturing files are not included in this candidate release. Do not infer interchangeability or commercial readiness from the prototype architecture.
