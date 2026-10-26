# Known limitations

- Waypoint field testing encountered aggressive heading acquisition and did not establish reliable autonomous navigation. Subsequent changes must be independently tested.
- Heading depends on sensor calibration, installation, nearby magnetic materials and current. Validate compass, gyro signs and GPS quality on the assembled vessel.
- Sensor calibration and battery ADC scaling must be checked against actual hardware; voltage-based battery percentages are estimates.
- FDM hull sealing, insert areas, cable glands and enclosure seals require inspection and water testing.
- CAN recovery, emergency stop and RC override require end-to-end validation on each build. Disconnect propulsion or remove propellers during bench tests.
- This is an experimental prototype; no production, safety certification or regulatory compliance is claimed.
