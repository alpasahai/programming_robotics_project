import math

def handle_fire_command(
  fsm, 
  bullet,
  pan_sensor,
  log,
):
    fire_command = fsm.consume_fire_command()
    
    if fire_command is not None:
        if bullet.is_parked:
        
        #Safety exclusion zone around the Search Radar:
            SEARCH_RADAR_BEARING_RAD = 0.0
            SAFETY_EXCLUSION_RAD = 0.4 #this is 23 degrees
            
            #Ensuring that it's [-pi, pi]
            pan_now = pan_sensor.getValue()
            pan_wrapped = math.remainder(pan_now, math.tau)
            
            #Smalled anglualr difference to the radar bearing
            angle_to_radar = abs(math.remainder(pan_wrapped - SEARCH_RADAR_BEARING_RAD, math.tau))
            
            fire_allowed = angle_to_radar > SAFETY_EXCLUSION_RAD
            
            if fire_allowed: 
                bullet.fire(fire_command)
                log.info(
                    "FIRE — bullet armed toward intercept [%.3f, %.3f, %.3f]",
                    fire_command[0],
                    fire_command[1],
                    fire_command[2],
                )
            else: 
                log.warning(
                    "FIRE SUPPRESSED - Search Radar exclusion zone"
                    "(pan=%.3f rad)", 
                    pan_wrapped,
                )
        else:
             log.warning("FIRE ignored — a bullet is still in flight")
             
             