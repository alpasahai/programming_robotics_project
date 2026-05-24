# ATLAS — Technical Requirements Compliance Map

Below is the complete evidence-mapped list. All citations are file:line in `controllers/atlas_controller/` unless noted.

---

## CORE REQUIREMENTS

### 1. ≥2 Inputs (sensors / perception) — **5 inputs**
- **Search Radar cue** (world-frame `[x,y,z]` over Webots radio) — `search_radar_link.py:66-75`
- **Fire-Control Radar (FCR)** turret-relative `[dx,dy,dz]`, gated by FOV + range — `fire_control_radar.py:81-106`
- **Ground-hit pulse** from attacker — `attacker_ground_hit_link.py:48-50`
- **Bullet-hit pulse** from attacker — `bullet_hit_link.py:49-51`
- **Pan/tilt position sensors** (measured aim; also used by the new safety exclusion gate) — `atlas_controller.py:133-136`

### 2. ≥2 Outputs (actuators / behaviour) — **4 outputs**
- **Pan motor command** — `fsm.py:495` `pan_motor.setPosition(target_pan)`
- **Tilt motor command** — `fsm.py:496` `tilt_motor.setPosition(tilt)`
- **Bullet fire/launch** (teleport-to-muzzle + velocity write) — `bullet.py:97-110`
- **Scoreboard updates** (operational logging) — `atlas_controller.py:314-316`

### 3. FSM
- `AtlasFSM` class in `fsm.py:81-614`; state constants `fsm.py:115-119`.

### 4. ≥4 Behavioural States — **5 states**
| State | Definition |
|---|---|
| **IDLE** `fsm.py:115,225-268` | Hold beam; wait for cues; pre-slew on AttackPredictor hint |
| **AIM** `fsm.py:116,270-303` | Reset filter; slew to cued world position; wait for FCR lock |
| **TRACK_PREDICT** `fsm.py:117,305-360` | Fuse Search+FCR; compute ballistic intercept; converge-gate |
| **ENGAGING** `fsm.py:118,362-392` | Hold aim at fixed intercept; await resolution |
| **RESET** `fsm.py:119,394-424` | Wipe filter, cue, counters; back to IDLE |

### 5. Multi-condition Decision Logic — **8 guards**
- **IDLE→RESET**: `ground_hit OR bullet_hit` — `fsm.py:239-244`
- **IDLE→AIM**: `_detection_count ≥ acquire_frames` (debounce) — `fsm.py:264`
- **AIM→RESET**: `ground_hit OR bullet_hit` — `fsm.py:287-292`
- **AIM→TRACK_PREDICT**: `fcr.is_locked()` (FOV ∧ range) — `fsm.py:302`
- **TRACK_PREDICT→RESET**: `ground_hit OR bullet_hit` — `fsm.py:327-332`
- **TRACK_PREDICT→ENGAGING** (triple-AND): `pred_error < threshold AND intercept in range AND _converge_count ≥ converge_frames` — `fsm.py:349-360`
- **ENGAGING→RESET**: `ground_hit OR bullet_hit OR NOT in_range` — `fsm.py:387-392`
- **FCR lock gate**: `separation ≤ fov_half AND range ≤ max_range` — `fire_control_radar.py:106`

### 6. Safety / Fail-safe Mechanisms — **8**
- **Universal RESET recovery** — `fsm.py:394-424`
- **Hit-cue precedence** (overrides every state) — `fsm.py:240,288,328`
- **Convergence gate** before firing (debounce + range) — `fsm.py:349-360`
- **Stale-cue clearing on RESET** — `fsm.py:417` + `search_radar_link.py:55-64`
- **Bullet flight-time timeout** (~400 steps recycle) — `atlas_controller.py:422`
- **FCR range > engagement range** (detect-before-shoot envelope) — `fire_control_radar.py:75,106`
- **Friendly-fire avoidance**: ground-hit only from attacker cue, never inferred from track Z — `fsm.py:373-375,577-589`
- **Search Radar exclusion zone** (NEW, merged PR #51, issue #48): even if the FSM commits to a shot, fire is suppressed when the turret pan is within ±`SAFETY_EXCLUSION_RAD = 0.4` rad (~23°) of the Search Radar bearing (`SEARCH_RADAR_BEARING_RAD = 0.0`). Pan is read from `pan_sensor.getValue()` and wrapped to `[-π, π]` via `math.remainder`; suppressed shots log at WARNING. Prevents the turret from destroying its own upstream sensor — `atlas_controller.py:399-434`

### 7. Structured System Architecture Diagram
- `README.md:10-19` four-layer Sense / Think / Act / Safety decomposition
- `docs/adr/0012-five-state-fsm-and-fsm-driven-dual-sensor-fusion.md:9-17` state table
- `report/fsm.pdf` FSM figure referenced in report
- Updated overall architecture diagram added in commit `af5a13d` (`ATLAS_architecture_diagram`) and refined in `805451d`
- Main-loop orchestration steps explicitly enumerated `atlas_controller.py:280-436`

---

## EMBEDDED INTELLIGENCE SPECIALISATION

### 8. Sensor Fusion
- **Kalman filter** (6-state CV + gravity): `track_filter.py:76-112` (`dim_x=6, dim_z=3, dim_u=3`; F kinematics; B with `u=[0,0,-9.81]`; Q process noise)
- **Heterogeneous noise weighting**: `R_FCR = 0.001` vs `R_SEARCH = 0.1` — `atlas_controller.py:95-96`
- **Per-sensor updates** (R swapped per source): `track_filter.py:127-159` `update_fcr()` / `update_search()`
- **Continuous predict, conditional update** (dropout-safe): `atlas_controller.py:330,338-340`
- **Filter reset on AIM entry** + `_initialised` gate before firing — `track_filter.py:161-180`

### 9. Context-aware Behaviour
- **AttackPredictor** online next-sector predictor — `attack_predictor.py:52-82`
- **Online logistic regression** (`SGDClassifier(loss="log_loss").partial_fit`) — `attack_predictor.py:64-70`, wired `atlas_controller.py:177-194`
- **Online sector discovery** (leader clustering, EMA-tracked centres, no preset count) — `sector_map.py:44-65`
- **Feature engineering** (one-hot current sector ⊕ run-length, modelling burst-switching) — `attack_features.py:13-33`
- **LaunchLatch** segmentation (one obs per engagement, avoids training on stale cue) — `launch_latch.py:15-48`
- **Pre-slew in IDLE** when prediction available — `fsm.py:250-256`
- **Adaptive firing readiness**: convergence gate adapts to track volatility (waits longer when filter unstable, fires sooner when stable) — `fsm.py:343-360` + config `fsm.py:72-78`

### 10. Real-time Logic
- **Synchronous Sense→Predict→Fuse→Decide loop** running every tick — `atlas_controller.py:289-435` (Step 1 sense lines 298-325; Step 2 predict 330; Step 3 fuse 338-340; Step 3b learn 347-388; Step 4 decide 391)
- **Webots tick = 32 ms**: `timestep = robot.getBasicTimeStep()` — `atlas_controller.py:123`; dt threaded into filter `atlas_controller.py:69`
- **Prediction horizon** scaled to tick: `lookahead_steps = 10` × 32 ms = 320 ms — `fsm.py:78`

### 11. Advanced AI / Adaptive Component (10%)
- **Online learning, no offline dataset**: `partial_fit` per launch — `attack_predictor.py:67-70`
- **Pattern discovery**: emergent sector structure via online angular clustering — `sector_map.py:1-65`
- **Drift adaptation**: EMA centre nudging (`nudge=0.2`) tracks slow drift — `sector_map.py:59-62`
- **Cold-start guard**: prediction blocked until ≥2 sectors observed — `attack_predictor.py:76-80`
- **A/B switch** for measuring adaptive value: `ATLAS_PREAIM` env flag — `atlas_controller.py:177-194`
- **Drift-recovery test**: `tests/test_attack_predictor.py:145-160` `adapts_after_drift`

---

**Summary**: All 7 core + 3 specialisation requirements have concrete code backing. The safety-mechanism count is now **8** with the merged FCR / Search Radar exclusion zone (PR #51) that suppresses any shot whose pan angle falls inside the ±23° wedge around the upstream Search Radar. The AI/adaptive 10% component is satisfied by the online logistic regression + online clustering pipeline (AttackPredictor / SectorMap / LaunchLatch) over a noisy, drifting launch-sector pattern.
