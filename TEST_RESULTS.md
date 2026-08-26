# ESP32-C5 ↔ Redmond RK-G211S test results

Test date: 2026-08-24  
Water volume: approximately 1 litre  
Initial water temperature: 21 °C

## Heating observations

- 40 °C program: the kettle reported 41 °C after approximately 6 min 08 s. The exact first 40 °C sample was lost from the serial-monitor backlog, so this duration is approximate.
- Changing an active target directly from 40 °C to 55 °C was ignored by the kettle. A reliable target change requires `OFF → SetMode → ON`.
- Thermal inertia after stopping near 50–51 °C: temperature continued from 51 °C to 56 °C over approximately 24 s (+5 °C).
- 70 °C program: after reconnecting at 66–67 °C it stabilized around 66–67 °C for a while, demonstrating the kettle's own control hysteresis.
- 85 °C program: 67→85 °C in 100.4 s (approximately 1 min 40 s). It then held 85 °C while the program remained active.
- Boil program: 85→100 °C in 38.4 s. Automatic switch-off occurred 40.5 s after the command.

## Power interruption / reconnect during heating

- The kettle was briefly removed from its base during the 70 °C program.
- Power loss was detected at ESP log time 600.880 s.
- BLE connection returned at 606.750 s.
- Authentication completed at 609.610 s.
- Autonomous idle backlight was restored at 610.110 s.
- First status after recovery arrived at 610.560 s: 66 °C, target cleared, heating off.
- Safety result: heating did not restart automatically after the kettle was returned to its base.

## Protocol conclusions

- `on=1` in a temperature program means the program is active; it does not prove that the heating element is continuously energized.
- For a new target while another program is active, always send `OFF`, then set the new mode/target, then send `ON`.
- After kettle power loss, reconnect and restore the autonomous idle-light configuration, but do not restore heating automatically.
