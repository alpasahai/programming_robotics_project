import math

#-----------------------------------------------------------------
# 1. Sense [sim-clock cadence]
#-----------------------------------------------------------------

def handle_sensor_updates(
    cue_link,
    ground_hit_link,
    bullet_hit_link,
    scoreboard,
    score_log,
    pan_sensor,
    tilt_sensor,
    fcr,
    robot,
    log,
):
    cue_link.update()
    ground_hit_link.update()
    bullet_hit_link.update()
    
    if ground_hit_link.hit_this_step():
        log.info(
            "[ATLAS] ground-hit cue received: hit #%d at t=%.2fs",
            ground_hit_link.count,
            robot.getTime(),
        )
    if bullet_hit_link.hit_this_step():
        log.info(
            "[ATLAS] bullet-hit cue received: hit #%d at t=%.2fs",
            bullet_hit_link.count,
            robot.getTime(),
        )
    if bullet_hit_link.hit_this_step():
        scoreboard.record_shoot_down()
    if ground_hit_link.hit_this_step():
        scoreboard.record_ground_hit()
    if bullet_hit_link.hit_this_step() or ground_hit_link.hit_this_step():
        score_log.info("t=%.1fs %s", robot.getTime(), scoreboard.summary())
    
    pan_actual = pan_sensor.getValue()
    tilt_actual = tilt_sensor.getValue()
    
    if math.isnan(pan_actual):
        pan_actual = 0.0
    if math.isnan(tilt_actual):
        tilt_actual = 0.0
    fcr.update(pan_actual, tilt_actual)
    
#-----------------------------------------------------------------
# 3b. Adaptive launcher observation (radar-only, segmented by resolution cues)
#-----------------------------------------------------------------
def handle_adaptive_observation(
    launch_latch,
    ground_hit_link,
    bullet_hit_link,
    cue_link,
    turret_position,
    attack_predictor,
    preaim_monitor,
    robot,
    log,
    last_prerotate_sector,
):
    observed_cue = launch_latch.update(
        resolved=ground_hit_link.hit_this_step() or bullet_hit_link.hit_this_step(),
        cue=cue_link.get_cue(),
    )
    if observed_cue is not None:
        rel_x = observed_cue[0] - turret_position[0]
        rel_y = observed_cue[1] - turret_position[1]
        launch_bearing = math.atan2(rel_x, rel_y)  # pan convention

        # Score the pre-aim we were holding for THIS launch (captured before the
        # online update changes the prediction), then learn from the launch.
        predicted_before = attack_predictor.predicted_sector
        ready_before = attack_predictor.get_ready_aim()
        predicted_bearing = None if ready_before is None else ready_before[0]
        actual_sector = attack_predictor.observe(launch_bearing, robot.getTime())
        preaim_monitor.record(predicted_before, actual_sector,
                              predicted_bearing, launch_bearing)

        if predicted_before is None:
            log.info(
                "[ATLAS] launch from sector %d (%.1f°) — cold start, no pre-aim yet | %s",
                actual_sector, math.degrees(launch_bearing), preaim_monitor.summary(),
            )
        else:
            hit = "HIT" if predicted_before == actual_sector else "MISS"
            err_deg = preaim_monitor.last_error_deg
            err_str = "n/a" if err_deg is None else "%.1f°" % err_deg
            log.info(
                "[ATLAS] launch from sector %d (%.1f°); pre-aimed sector %d (%s, err %s) | %s",
                actual_sector, math.degrees(launch_bearing),
                predicted_before, hit, err_str, preaim_monitor.summary(),
            )

        # Announce a change in the pre-rotation target for the NEXT launch.
        next_aim = attack_predictor.get_ready_aim()
        next_sector = attack_predictor.predicted_sector
        if next_aim is not None and next_sector != last_prerotate_sector:
            log.info(
                "[ATLAS] pre-rotating toward sector %d (bearing %.1f°) for next launch",
                next_sector, math.degrees(next_aim[0]),
            )
            last_prerotate_sector = next_sector
            
    return last_prerotate_sector