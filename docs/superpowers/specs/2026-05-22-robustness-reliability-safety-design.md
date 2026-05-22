# Robustness, Reliability & Safety — design exploration

**Issue:** #49 — *spec: design, spec and plan the safety, robustness and reliability features.*
**Status:** Draft / idea catalogue for review.
**Assignment hook:** Marking criterion **"Safety, Robustness & Reliability — 10%"** —
*"Inclusion of appropriate safety mechanisms and stable operation under different
conditions."* (`assignment-spec-3003ict-project.pdf`, §8.a).

This document (1) inventories what ATLAS *already* implements against that
criterion and justifies each item, (2) catalogues new ideas — Madeline's first
and attributed, then Claude's — with a difficulty-vs-contribution score, and
(3) recommends a go-forward set sized for the current time crunch.

---

## 0. Context the ideas have to respect

A few facts from the code that constrain what counts as "robustness" here:

- **The radars are not vision sensors.** Both the Fire-Control Radar
  (`fire_control_radar.py`) and the external Search Radar (`search_radar.py`)
  derive target position from Webots `Supervisor.getPosition()` (ground truth)
  plus **Gaussian noise** and a **FOV-cone + range gate**. There is no `Camera`
  node and no image pipeline anywhere in the controllers. *Radar is, by physics,
  insensitive to ambient light.* This directly shapes idea **M3** below.
- **The turret fires a real, travelling bullet** (`bullet.py`, ADR-0013) along
  its aim — not an instant-hit laser. So a bad aim genuinely launches a physical
  object in that direction. This is what makes a *friendly-fire interlock* (idea
  **M1**) meaningful rather than cosmetic.
- **The FSM is already the "SAFETY" module** (`fsm.py`). It owns every state
  transition and is fully unit-tested, so new safety logic has a clean,
  test-friendly home — it does not need to be sprinkled across the controller.
- **The Kalman filter already runs every tick** (`track_filter.predict()` in
  `atlas_controller.py`, ADR-0003 continuous fusion), so it can *coast* (predict
  with no measurement) for free — the backbone of any sensor-failure fallback
  (idea **M2**).

---

## 1. What ATLAS already implements (with justification)

These are real, in-tree mechanisms that already earn marks under the criterion.
The report/video should claim them explicitly.

| # | Feature | Category | Where | What it does / why it counts |
|---|---------|----------|-------|------------------------------|
| E1 | **RESET state** wipes the Kalman filter, cue, and all engagement bookkeeping | Reliability | `fsm.py:379-409` | Central fail-safe: any abnormal end-of-engagement returns the system to a known-clean `IDLE`. This *is* the pitch's "automatic reset" promise. |
| E2 | **Out-of-range auto-reset** — `_target_within_range()` forces RESET when target/intercept leaves `max_range` (10 m) | Reliability | `fsm.py:558-574`, `:372-377` | Directly satisfies the pitch's "reset when the object is lost / no longer in range". |
| E3 | **Ground-hit / bullet-hit cues pre-empt every active state → RESET** | Safety | `fsm.py:233-238, 272-277, 312-317, 372-377` | The turret stops engaging the instant the threat is resolved or has landed — no firing at a dead target. |
| E4 | **Kalman filter smooths noisy sensor measurements** | Reliability | `track_filter.py` | The pitch's "explore filtering sensor noise" — already done, with per-sensor noise covariances (`R_fcr` ≪ `R_search`). |
| E5 | **Acquire debounce** — `acquire_frames=3` consecutive cues before leaving IDLE | Robustness | `fsm.py:55, 249-253` | Rejects single-frame spurious cues; won't slew on a one-off blip. |
| E6 | **Fire-convergence gate** — `converge_frames=3` sub-threshold prediction-error steps before ENGAGING | Safety | `fsm.py:334-345` | Fire discipline: the turret will not shoot until its *own past predictions* have proven accurate. Prevents wild shots on an unconverged track. |
| E7 | **Pan-seam unwrap** prevents the turret whipping a near-360° turn | Robustness | `fsm.py:496-521` | Stable, predictable mechanical motion — "stable operation" in the literal sense. |
| E8 | **Atomic dual-motor command** — pan & tilt always commanded together | Robustness | `fsm.py:453-494` | No half-slewed states where one axis lags the other. |
| E9 | **NaN guard on joint position sensors** (fall back to 0.0) | Robustness | `atlas_controller.py:249-252` | Survives the unsettled first-tick sensor read without crashing. |
| E10 | **Bullet max-lifetime timeout** (`BULLET_MAX_LIFETIME_STEPS=400`) recycles a never-resolving bullet | Safety | `atlas_controller.py:92, 299-305` | No orphaned/leaked projectiles; bounded engagement duration. |
| E11 | **"One bullet at a time" interlock** — fire ignored while a bullet is in flight, logged | Safety | `atlas_controller.py:283-293` | A fire-rate interlock — can't double-fire. |
| E12 | **FCR FOV-cone + range gate** — only locks/fuses targets inside the beam | Robustness | `fire_control_radar.py:106-117` | Rejects off-axis returns; the filter is never fed a target the sensor can't actually see. |
| E13 | **Search-radar track-timeout buffer** holds a track across beam passes, drops it after N misses | Robustness | `search_radar.py:208-258` | Tolerates momentary loss of detection (object briefly disappears) without dropping lock instantly. |
| E14 | **Fail-fast on missing scene nodes** (`RuntimeError` if DEF not found) | Reliability | `atlas_controller.py:131, 154` | Misconfiguration surfaces immediately, not as silent wrong behaviour. |
| E15 | **Stale-cue clearing on RESET** (cue link has no expiry; RESET clears it) | Reliability | `search_radar_link.py:55-64`, `fsm.py:402` | Prevents a leftover cue from re-triggering an engagement against a phantom target. |

**Headline:** the pitch lists three S/R/R promises. E1/E2/E4/E15 already deliver
the *Reliability* promise. **The pitch's stated *Safety* promise — "if the
motors operate above a certain movement frequency, the system enters RESET" — is
NOT implemented** (see W1 below). That is the most important gap to close,
because the team has already written it down as a deliverable.

### Partially implemented / scaffolded

- **W1 — Motor over-frequency → RESET:** promised in the pitch, **not in code**.
  No watchdog on slew rate or command thrash anywhere. → addressed by idea **C1**.
- **W2 — Multi-object prioritisation:** the Search Radar maintains a multi-track
  buffer and selects one target internally, but ATLAS only ever receives a
  single cue position (`SearchRadarLink`). There is no *explicit, justifiable*
  prioritisation policy (nearest? soonest-impact?). → idea **C5**.
- **W3 — Cue freshness timeout:** explicitly flagged as a deliberate non-feature
  in `search_radar_link.py:7-9` ("freshness timeout … intentionally NOT
  implemented"). A known, documented hook. → idea **C2**.

---

## 2. Madeline's ideas (author: Madeline)

> These three were proposed by Madeline directly. Captured verbatim in intent,
> then assessed.

### M1 — Friendly-fire prevention: make it impossible for the FCR/turret to fire at the Search Radar

**Idea:** the turret must never launch a bullet in a direction that would hit a
friendly asset (the Search Radar node sitting in the world).

**Assessment:** Excellent fit, and the strongest demo of the three. The Search
Radar's world position is known to ATLAS (it's a fixed node). Add a **keep-out
angular sector** (a "restrictive fire line" / forbidden sector — exactly the
fratricide-prevention measure real fire-control doctrine uses) around the
friendly asset's bearing. The fire-inhibit interlock vetoes the
`TRACK_PREDICT → ENGAGING` transition (or zeroes the fire command) whenever the
intercept bearing falls inside that cone. This is genuine **multi-condition
decision logic** + a **safety interlock**, both explicitly rewarded by the
rubric, and it's visually obvious in the video ("watch — it refuses to shoot
when the threat lines up with our own radar").

- **Difficulty:** Low–Medium. New pure helper on the FSM + one transition guard;
  fully unit-testable with stub bearings. No new Webots nodes required.
- **Contribution:** High. Closes a safety story the rubric loves and demos well.

### M2 — Sensor-failure fallback: radars operate independently via Kalman modes

**Idea:** if a sensor dies, the system degrades gracefully instead of stalling:
(a) if the FCR can't get cues from the Search Radar, the FCR runs a **backup
search sweep** to find the target itself; (b) if the Search Radar dies or FCR
perception fails, the turret aims from whichever cue source is still alive.

**Assessment:** This is textbook **graceful degradation** (the same principle
AESA radars and UAV fire-control systems use — degrade capability, keep the
safety/mission function alive). It is the highest-value robustness idea because
it exercises the existing dual-sensor fusion *and* the always-warm Kalman filter
(which can coast on `predict()` alone). Concretely, three fallback behaviours:

1. **No cue for N steps in IDLE → autonomous backup PAN sweep** (instead of
   holding the fixed idle pose), so ATLAS can self-acquire when the Search Radar
   is offline. Replaces today's "stuck pointing at the sky" behaviour.
2. **FCR lock lost mid-track → coast on the Kalman prediction** (dead-reckoning)
   for a bounded number of steps, and/or fall back to **cue-only aiming**, before
   giving up to RESET. The filter already predicts every tick, so this is mostly
   wiring + a degraded-mode flag.
3. **Search Radar silent during track → continue on FCR alone** (already the
   nominal path once locked) — making this explicit + logged demonstrates the
   single-sensor degraded mode.

- **Difficulty:** Medium. Needs a "sensor health / staleness" notion and 1–2 new
  FSM sub-behaviours (a backup-sweep mode and a coast timer). Leverages existing
  Kalman + fusion, so no new estimator work.
- **Contribution:** Very High. Hits Robustness *and* Reliability *and* doubles as
  evidence for the "sensor fusion / real-time logic" specialisation marks.

### M3 — Testing under different lighting / visibility conditions

**Idea:** the radar should be robust to changing lighting and poor visibility.

**Assessment — important caveat:** *as built, the radars don't use light at all*
(see §0 — they read `getPosition()` + Gaussian noise). Changing the Webots scene
lighting would have **zero effect** on detection, so a literal "lighting test"
wouldn't demonstrate anything. Two honest ways to honour the intent:

- **(Recommended) Reframe as environmental-condition robustness for a radar:**
  build a **robustness test matrix** that varies the things radar actually cares
  about — elevated sensor noise (σ), cue dropout rate, process noise / wind-drag
  disturbance, and target speed — and show the Kalman filter + gating + RESET keep
  ATLAS stable. This *is* "stable operation under different conditions" in the
  rubric's words, costs almost no new code, and gives the report hard numbers.
- **(Only if time allows) Add a real `Camera` + simple vision detector** as a
  *third* perception source, then lighting genuinely matters. This is a large
  change to the sensing model and is **not advised under the current time
  crunch**, but it's the only path where "lighting" is literally true.

- **Difficulty:** Low (test matrix) / High (real camera).
- **Contribution:** Medium (test matrix — great for report + video, low novelty) /
  High-but-risky (camera).

---

## 3. Claude's additional ideas

### C1 — Motor over-slew / command-thrash watchdog → RESET *(closes the pitch's own promise)*

A watchdog that trips RESET when the commanded slew rate exceeds a limit, or when
the pan command reverses sign repeatedly within a short window (thrash). This is
**exactly the pitch's stated Safety mechanism** ("motors above a movement
frequency → RESET"), currently unbuilt (W1). Watchdog-drives-system-to-safe-state
is a standard embedded-safety pattern.
- **Difficulty:** Low. A small rate/thrash detector feeding the existing RESET.
- **Contribution:** High — it converts a *promised-but-missing* deliverable into a
  real one, which assessors will check against the pitch.

### C2 — Cue freshness timeout in `SearchRadarLink` *(closes a documented hook)*

Drop a stale cue after N steps of radio silence (the code itself flags this as a
deliberate omission, W3). Prevents acting on a phantom target after the Search
Radar dies — and is the natural trigger for M2's degraded modes.
- **Difficulty:** Low. A counter + a step input on `get_cue()`. Already scoped by
  the existing docstring.
- **Contribution:** Medium-High (also an enabler for M2).

### C3 — Fire-solution validity gate (no shooting the ground / behind / over limits)

Beyond range (E2), reject intercepts that are below ground, behind the turret, or
outside the tilt envelope before allowing ENGAGING. More multi-condition decision
logic; pairs naturally with M1 as one "is this shot allowed?" predicate.
- **Difficulty:** Low. Pure predicate on the intercept, unit-testable.
- **Contribution:** Medium.

### C4 — Covariance-based track-quality gate

Use the Kalman covariance `P` (already maintained) to refuse firing while
positional uncertainty is high — a more principled companion to the
`converge_frames` heuristic (E6). Shows real understanding of the filter.
- **Difficulty:** Medium. Read `P`, pick a trace/eigenvalue threshold, tune.
- **Contribution:** Medium-High (strong "intelligence" talking point in the viva).

### C5 — Explicit multi-target prioritisation policy

Make the Search Radar's target selection an explicit, justifiable rule —
**soonest-impact** (lowest predicted time-to-ground) or **nearest** — instead of
an implicit pick. Honours the pitch's "multiple objects → prioritisation".
- **Difficulty:** Medium (needs a clear rule + maybe multiple projectiles in-scene
  to demo).
- **Contribution:** Medium. Good if the demo world will actually have >1 target;
  low value if it won't.

### C6 — Tilt soft-limit / mechanical envelope enforcement

The code notes pan has no limits; enforce soft limits and refuse out-of-envelope
aim commands. Mechanical-safety story.
- **Difficulty:** Low.
- **Contribution:** Low-Medium (somewhat overlaps C3).

---

## 4. Scorecard

Effort = engineering cost under time crunch (lower is better). Value = marks +
demo/viva strength. ★ = relative.

| Idea | Effort | Value | Closes a promise? | Notes |
|------|:------:|:-----:|:-----------------:|-------|
| **M1** Friendly-fire keep-out | ★★ | ★★★★ | new safety story | Flagship safety demo |
| **M2** Sensor-failure graceful degradation | ★★★ | ★★★★★ | reliability + fusion | Flagship robustness demo |
| **M3a** Robustness test matrix (reframed) | ★ | ★★★ | "different conditions" | Cheap report/video win |
| **M3b** Real camera + lighting | ★★★★★ | ★★★★ | — | **Too risky now** |
| **C1** Motor watchdog → RESET | ★ | ★★★★ | **pitch Safety promise** | Cheapest high-value item |
| **C2** Cue freshness timeout | ★ | ★★★ | code hook + enables M2 | Do alongside M2 |
| **C3** Fire-solution validity gate | ★ | ★★ | — | Bundle with M1 |
| **C4** Covariance fire gate | ★★★ | ★★★ | — | Best viva talking point |
| **C5** Multi-target prioritisation | ★★★ | ★★ | pitch Robustness (partial) | Only if demo has >1 target |
| **C6** Tilt soft-limits | ★ | ★★ | — | Overlaps C3 |

---

## 5. Recommendation (go-forward set for the time crunch)

Ship a **tight, high-leverage bundle** that closes the written promises and demos
clearly, and explicitly *defer* the expensive items.

**Tier 1 — do these (high value, low/medium effort):**
1. **C1 — Motor over-slew watchdog → RESET.** Cheapest way to turn a
   *promised-but-missing* deliverable into a real one. Assessors will diff the
   pitch against the build; this removes a guaranteed ding.
2. **M1 — Friendly-fire keep-out interlock**, implemented together with **C3**
   (fire-solution validity gate) as one "is this shot permitted?" predicate.
   Flagship safety demo, low-medium effort, very strong on video and in the viva.
3. **M2 (scoped) — Graceful degradation**, built on **C2** (cue freshness
   timeout) as its trigger. Minimum viable version: *cue-loss → coast on Kalman
   for N steps → backup PAN sweep → RESET*. This is the single best Robustness +
   Reliability story and reuses the existing filter/fusion.

**Tier 2 — do if time remains:**
4. **M3a — Robustness test matrix** (noise σ, dropout, disturbance, speed). Almost
   free, gives the report hard numbers, and is the honest version of M3.
5. **C4 — Covariance fire gate.** Best single "we understand our Kalman filter"
   talking point for the individual-understanding marks.

**Defer / avoid under time pressure:**
- **M3b (real camera + lighting):** changes the whole sensing model — out of scope
  for the deadline. Document *why radar is light-insensitive* in the report
  instead; that's a stronger answer than a fake lighting test.
- **C5 (multi-target prioritisation):** only worth it if the demo world will
  actually contain multiple simultaneous targets. Otherwise it's invisible.
- **C6:** fold into C3 rather than building separately.

**Why this set:** it (a) closes both pitch promises that are currently unbuilt or
partial (C1 for Safety, M2/C2 for Reliability), (b) leads with the two demos that
read clearly on a 3-minute video (M1 refusing a fratricidal shot; M2 surviving a
killed sensor), and (c) avoids the only genuinely expensive idea (M3b). Total
Tier-1 effort is small because every item attaches to an existing, tested seam —
the FSM transitions, the RESET handler, and the always-running Kalman filter.

---

## Sources

- Graceful degradation in radar / fire-control: [Defence Science Journal — Graceful Degradation, Airborne Surveillance Radar](https://publicationsdrdo.in/index.php/dsj/article/view/12135); [ScienceDirect — Sensor fusion & adaptive control for UAV fire control, graceful degradation under adverse conditions](https://www.sciencedirect.com/science/article/pii/S2090447925003545); [Risknowlogy — Degraded Mode in Safety Systems](https://risknowlogy.com/articles/detail/17309/)
- Fratricide prevention / sectors of fire / restrictive fire line: [US Army FM 3-21.8 — Infantry Rifle Platoon and Squad](https://www.marines.mil/Portals/1/Publications/FM%203-21.8%20%20The%20Infantry%20Rifle%20Platoon%20and%20Squad_2.pdf); [US Patent 9,772,155 — Prevention of friendly fire incidents](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9772155)
- Watchdog-to-safe-state pattern: [Ganssle — Designing Great Watchdog Timers for Embedded Systems](https://www.ganssle.com/watchdogs.htm)
