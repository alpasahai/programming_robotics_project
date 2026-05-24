#figure(
  text(size: 6pt)[
```python
# atlas_controller.py — anti-friendly-fire gate before arming the bullet
fire_command = fsm.consume_fire_command()
if fire_command is not None and bullet.is_parked:
    SEARCH_RADAR_BEARING_RAD = 0.0
    SAFETY_EXCLUSION_RAD = 0.4  # ~23 degrees

    pan_now = pan_sensor.getValue()
    pan_wrapped = math.remainder(pan_now, math.tau)
    angle_to_radar = abs(
        math.remainder(pan_wrapped - SEARCH_RADAR_BEARING_RAD, math.tau)
    )

    if angle_to_radar > SAFETY_EXCLUSION_RAD:
        bullet.fire(fire_command)
    else:
        log.warning("FIRE SUPPRESSED - Search Radar exclusion zone")
```
  ],
  caption: [Pan-angle gate: shot is suppressed if the boresight is within $plus.minus 0.4$ rad of the Search Radar.],
) <code:friendly-fire>
