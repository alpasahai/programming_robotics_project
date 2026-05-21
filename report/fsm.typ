#figure(
  table(
    columns: 2,
    align: (center, left),
    [State], [Purpose],
    [`IDLE`],
    [
      - `FCR` is waiting for a cue from `Search Radar`
      - The turret idles with its search beam pointing in some specified direction.
      - *Exit conditions*:
        - A cue is received from `Search Radar` (go to `AIM`)
        // TODO: this is unnecessary transition but im just putting it here for completeness.
        - A ground hit cue is received by `atlas_controller` (go to `RESET`)
    ],

    [`AIM`],
    [
      - After receiving a cue from `Search Radar`, but before finding the target itself,
        the `FCR` turret uses the most recent `Search Radar` updates to determine where
        to slew to.
        - This may involve some kind of predictive or filtering feature if it is found
          experimentally that the `Search Radar` sensor noise is too unreliable or the
          projectile moves too quickly for the `FCR` to lock onto. Only add this
          complexity if it is deemed necessary.
      - This state is where the `FCR` is attempting to lock its tracking beam onto the
        target. The turret is moving.
      - No sensor fusion happens here because the only valid data points are the ones
        coming from the `Search Radar`.
      - *Exit conditions*:
        - The `FCR` beam finds the target (go to `TRACK & PREDICT`)
        - A ground hit cue is received by `atlas_controller` (go to `RESET`)
    ],

    [`TRACK & PREDICT`],
    [
      - `FCR` is actively performing prediction using the Kalman filter which implements
        sensor fusion of the `Search Radar` and the `FCR` data points. This filter
        continually updates to make predictions on where the projectile will be in some
        timestep.
      - *Exit conditions*:
        - The error between the predicted value and the observed value closes enough to
          justify making a prediction and firing a bullet (go to `ENGAGING`)
        - A ground hit cue is received by `atlas_controller` (go to `RESET`)
    ],

    [`ENGAGING`],
    [
      - The turret fires a bullet at the projectile based off the prediction it made.
      - No more predictions are happening here, the tracking can end.
      - *Exit conditions*:
        - A projectile destroyed cue is received by `atlas_controller` (go to `RESET`)
        - A ground hit cue is received by `atlas_controller` (go to `RESET`)
    ],

    [`RESET`],
    [
      - Currently I'm not sure if there's really any reset logic, but maybe the Kalman
        filter memory should be wiped here (we may end up with multiple Kalman filters)

      - *Exit conditions*:
        - System reset complete (go to `IDLE`)
    ],
  ),
  caption: [ATLAS Finite state machine states],
)


