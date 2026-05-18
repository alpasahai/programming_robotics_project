# Turret Weapon — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ATLAS turret fire a physical, self-reporting projectile along its aim direction when the FSM reaches `ENGAGING`.

**Architecture:** The FSM emits a one-shot *fire command* on entering `ENGAGING` and spawns nothing itself — staying pure and unit-testable. `atlas_controller` (already a Webots supervisor) consumes that command, dynamically imports an `AtlasBullet` node at the turret muzzle, and launches it with `setVelocity()`. The bullet is a `Robot` carrying a `bumper` `TouchSensor` and a minimal controller that publishes a hit via `customData`; the turret supervisor reads that field, resolves the shot, and removes the node.

**Tech stack:** Python 3.13 (see `.python-version` / `pyproject.toml`), pytest, Webots R2025a. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-18-turret-weapon-design.md`.

---

## How to work this plan

- **Execution mode: subagent-driven.** An Opus orchestrator dispatches one fresh subagent per task, reviews the result, then dispatches the next. Use the `superpowers:subagent-driven-development` sub-skill.
- Each task is one reviewable unit. For the unit-testable task (Task 1) follow red-green-refactor: write the failing test, confirm it fails for the expected reason, implement the minimum, confirm green, commit.
- Run tests from `controllers/atlas_controller/`: `python -m pytest tests/ -v`. `conftest.py` puts both `tests/` and the controller dir on `sys.path`.
- **Webots glue is not unit-tested** — `atlas_controller.py`, the new bullet controller, and the `.proto`/`.wbt` files match the project's existing pattern (the controller is documented as untested glue). They get explicit manual-verification steps in Webots instead. Proto/world *structure* is covered by the regression test in `tests/test_world_proto_layout.py`.
- Commit after every task with a conventional-commit message.

### Model and effort policy

Each task carries a **Model · Effort** tag for the subagent that executes it. Goal: cheapest model and lowest reasoning effort that will not degrade quality.

**Model** — capability tier:
- **haiku** (Codex: `gpt-5.4-mini`) — fully specified, mechanical, self-contained.
- **sonnet** (Codex: `gpt-5.4`) — moderate reasoning: logic within a clear spec, Webots/proto syntax.
- **opus** (Codex: `gpt-5.5`) — judgment calls, cross-cutting changes, ambiguity. The orchestrator is always Opus, and every between-task review is done by Opus regardless of the task's execution model.
- **Codex orchestration:** If you're Codex and you're orchestrating this plan, use the Codex models instead: `gpt-5.5` for the orchestrator/reviews, `gpt-5.4` where the plan says `sonnet`, and `gpt-5.4-mini` where the plan says `haiku`.

**Effort** — how much reasoning the subagent spends, set independently of model:
- **low** — transcription-level: the plan dictates the code/content; follow it.
- **medium** — the subagent designs small pieces of logic within a clear spec.
- **high** — the subagent reasons through edge cases or cross-file effects. Rare in this plan.

**Rules:**
- **Webots parameter tuning is never delegated to haiku.** `MUZZLE_SPEED`, `MUZZLE_OFFSET_M`, bullet size, and the flight-time safety timeout are iterative judgement — the orchestrator (Opus) or the human tunes them, even within a sonnet-tagged task.
- If a subagent on a cheaper model or lower effort hits ambiguity or finds the task under-specified, it stops and escalates to the orchestrator rather than guessing.

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `controllers/atlas_controller/fsm.py` | modify | Add the one-shot `fire_command` (set on `→ ENGAGING`) and `consume_fire_command()`. |
| `controllers/atlas_controller/tests/test_fsm.py` | modify | Tests for the fire command. |
| `protos/AtlasBullet.proto` | create | Webots PROTO for the bullet — a `Robot` with a sphere body, physics, a `bumper` `TouchSensor`, and the bullet controller. |
| `worlds/ATLA_v1.wbt` | modify | Add `IMPORTABLE EXTERNPROTO "../protos/AtlasBullet.proto"` so the supervisor can spawn it. No instance is placed. |
| `controllers/atlas_controller/tests/test_world_proto_layout.py` | modify | Regression assertions for `AtlasBullet.proto` and the importable declaration. |
| `controllers/atlas_bullet_controller/atlas_bullet_controller.py` | create | Minimal controller: read the `TouchSensor`, publish `"HIT"` to `customData` on first contact. |
| `controllers/atlas_controller/atlas_controller.py` | modify | Consume the fire command; spawn, launch, and manage the bullet's lifecycle. |
| `docs/adr/0008-turret-fired-projectile-weapon.md` | create | Records the decision. |

`CONTEXT.md` was already updated when the spec was committed (`Turret Weapon`, `Bullet`, `Fire Command` glossary entries) — this plan does not touch it.

---

## Task 1 — FSM fire command

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — small, well-bounded state-machine change; the only unit-tested task, so the interface must be right.

**Required:** The controller needs to know the exact moment the FSM commits to a shot and where to aim it. The FSM must signal this *once* per engagement and must not spawn anything itself (node spawning is Webots-specific and untestable — it belongs in the controller).

**Current state:** `fsm.py` — `_transition()` resets per-state bookkeeping for the entered state. `_do_aim()` calls `_transition(self.ENGAGING)` once aim error is below threshold; at that point `self._intercept` holds the validated intercept (relative `[dx, dy, dz]` from the turret). `__init__` initialises per-state bookkeeping near `self.laser_active = False`.

**What to build:**
- In `__init__`, add `self.fire_command = None` (alongside `self.laser_active = False`). Document it: "Set to the intercept `[dx, dy, dz]` for one step when the FSM enters ENGAGING; consumed and cleared by the controller via `consume_fire_command()`."
- In `_transition()`, add an `elif new_state == self.ENGAGING:` branch that sets `self.fire_command = list(self._intercept)`. (`_intercept` is guaranteed set — `PREDICT` sets it before `AIMING`, and `AIMING` is the only path to `ENGAGING`.)
- Add a method `consume_fire_command()`:

```python
def consume_fire_command(self):
    """Return the pending fire command and clear it, or None if none pending.

    The fire command is the intercept point [dx, dy, dz] (turret-relative
    metres) the FSM committed to when it entered ENGAGING. It is a one-shot
    signal: the controller calls this once per step after fsm.step(); the
    command is returned exactly once and is None on every subsequent call
    until the FSM re-enters ENGAGING.
    """
    command = self.fire_command
    self.fire_command = None
    return command
```

- In `_do_reset()`, add `self.fire_command = None` alongside the other explicit clears, so a fire command never survives a RESET unconsumed.

**Approach (TDD):**

- [ ] **Step 1: Write the failing tests** — add to `controllers/atlas_controller/tests/test_fsm.py`:

```python
# ---------------------------------------------------------------------------
# Fire command — emitted once on entering ENGAGING
# ---------------------------------------------------------------------------

def _drive_to_engaging(fsm, intercept):
    """Put fsm into AIMING with a fixed intercept, then step into ENGAGING.

    _do_aim measures error against the previous commanded angles, so the
    first step stays in AIMING and the second transitions to ENGAGING.
    """
    fsm.state = AtlasFSM.AIMING
    fsm._intercept = list(intercept)
    fsm.step()  # still AIMING — error measured against stale commanded angles
    fsm.step()  # error now zero → ENGAGING


def test_no_fire_command_before_engaging():
    """fire_command is None until the FSM enters ENGAGING."""
    fsm, _, _ = _make_fsm()
    assert fsm.fire_command is None
    assert fsm.consume_fire_command() is None


def test_entering_engaging_sets_fire_command_to_intercept():
    """Transitioning into ENGAGING records the intercept as the fire command."""
    fsm, _, _ = _make_fsm()
    _drive_to_engaging(fsm, [2.0, 3.0, 1.5])
    assert fsm.state == AtlasFSM.ENGAGING
    assert fsm.fire_command == [2.0, 3.0, 1.5]


def test_consume_fire_command_returns_then_clears():
    """consume_fire_command returns the command once, then None."""
    fsm, _, _ = _make_fsm()
    _drive_to_engaging(fsm, [2.0, 3.0, 1.5])
    assert fsm.consume_fire_command() == [2.0, 3.0, 1.5]
    assert fsm.consume_fire_command() is None


def test_reset_clears_an_unconsumed_fire_command():
    """A fire command never survives a RESET."""
    fsm, _, _ = _make_fsm()
    _drive_to_engaging(fsm, [2.0, 3.0, 1.5])
    fsm.state = AtlasFSM.RESET
    fsm.step()
    assert fsm.fire_command is None


def test_re_entering_engaging_emits_a_fresh_fire_command():
    """A second engagement emits its own fire command."""
    fsm, _, _ = _make_fsm()
    _drive_to_engaging(fsm, [2.0, 3.0, 1.5])
    assert fsm.consume_fire_command() == [2.0, 3.0, 1.5]
    _drive_to_engaging(fsm, [-1.0, 4.0, 0.8])
    assert fsm.consume_fire_command() == [-1.0, 4.0, 0.8]
```

- [ ] **Step 2: Run the tests, confirm they fail**

Run: `cd controllers/atlas_controller && python -m pytest tests/test_fsm.py -k fire -v`
Expected: FAIL — `AttributeError: 'AtlasFSM' object has no attribute 'fire_command'` / `consume_fire_command`.

- [ ] **Step 3: Implement the change** in `fsm.py` — add `self.fire_command = None` in `__init__`, the `ENGAGING` branch in `_transition()`, the `consume_fire_command()` method, and the clear in `_do_reset()`, exactly as described above.

- [ ] **Step 4: Run the tests, confirm green**

Run: `cd controllers/atlas_controller && python -m pytest tests/ -v`
Expected: PASS — the new fire-command tests pass and every existing FSM/filter/predictor test stays green (the change is purely additive).

- [ ] **Step 5: Commit**

```bash
git add controllers/atlas_controller/fsm.py controllers/atlas_controller/tests/test_fsm.py
git commit -m "feat(fsm): emit a one-shot fire command on entering ENGAGING"
```

---

## Task 2 — `AtlasBullet.proto` and the importable declaration

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** — Webots PROTO/`.wbt` syntax. Inspect `protos/SimulatedProjectile.proto` and `protos/AtlasTurret.proto` first to copy header and field conventions.

**Required:** The turret needs a projectile template to spawn. It must be a `Robot` (not a plain `Solid`) because it carries a `TouchSensor` device and a controller. It is never pre-placed — `IMPORTABLE EXTERNPROTO` makes it spawnable at runtime via `importMFNodeFromString`.

**What to build — `protos/AtlasBullet.proto`:**

```
#VRML_SIM R2025a utf8
# The turret's fired projectile: a small, self-reporting sphere.
# A Robot (not a Solid) so it can carry a TouchSensor + controller.
# Spawned dynamically by the turret supervisor; never pre-placed.

PROTO AtlasBullet [
  field SFVec3f translation 0 0 0
  field SFString name "TURRET_BULLET"
  field SFString controller "atlas_bullet_controller"
] {
  Robot {
    translation IS translation
    children [
      DEF BULLET_SHAPE Shape {
        appearance PBRAppearance {
          baseColor 0 0.4 1
          emissiveColor 0 0.2 0.6
          roughness 0.3
          metalness 0
        }
        geometry Sphere {
          radius 0.12
          subdivision 3
        }
      }
      TouchSensor {
        name "BULLET_TOUCH"
        type "bumper"
        boundingObject USE BULLET_SHAPE
      }
    ]
    name IS name
    model "atlas_bullet"
    boundingObject USE BULLET_SHAPE
    physics Physics {
      density -1
      mass 0.2
    }
    controller IS controller
    supervisor FALSE
  }
}
```

Notes for the implementer:
- The blue colour keeps the bullet visually distinct from the red incoming `SimulatedProjectile`.
- The `Robot` node has a built-in `customData` `SFString` field — the bullet controller writes it and the turret supervisor reads it. It does **not** need declaring in the PROTO interface.
- `radius 0.12` and `mass 0.2` are starting values; the orchestrator/human may tune them during Task 6 verification (a larger radius reduces tunelling).

**What to change — `worlds/ATLA_v1.wbt`:** Add this line to the `EXTERNPROTO` block (near the existing `EXTERNPROTO "../protos/AtlasTurret.proto"`), using `IMPORTABLE` so the node can be spawned at runtime:

```
IMPORTABLE EXTERNPROTO "../protos/AtlasBullet.proto"
```

Do **not** add a `DEF ... AtlasBullet` instance — the bullet only ever exists when the supervisor spawns one.

**What to change — `controllers/atlas_controller/tests/test_world_proto_layout.py`:** Add a regression test:

```python
BULLET_PROTO = REPO_ROOT / "protos" / "AtlasBullet.proto"


def test_atlas_bullet_proto_is_importable_and_well_formed():
    """The bullet PROTO exists, is a sensor-carrying Robot, and is importable."""
    world = WORLD_FILE.read_text(encoding="utf-8")
    bullet_proto = BULLET_PROTO.read_text(encoding="utf-8")

    # Importable so the supervisor can spawn it; never pre-placed as an instance.
    # An instance would read "AtlasBullet {"; the EXTERNPROTO line reads
    # "AtlasBullet.proto", so this distinguishes the two.
    assert 'IMPORTABLE EXTERNPROTO "../protos/AtlasBullet.proto"' in world
    assert "AtlasBullet {" not in world

    assert "PROTO AtlasBullet" in bullet_proto
    assert "Robot {" in bullet_proto                       # Robot, not Solid
    assert 'name "BULLET_TOUCH"' in bullet_proto           # the TouchSensor
    assert 'type "bumper"' in bullet_proto
    assert 'controller "atlas_bullet_controller"' in bullet_proto
    assert "supervisor FALSE" in bullet_proto
```

**Verification:**

- [ ] Run `cd controllers/atlas_controller && python -m pytest tests/test_world_proto_layout.py -v` — the new test passes, existing layout tests stay green.
- [ ] Open `worlds/ATLA_v1.wbt` in Webots: it loads with no PROTO parse errors in the console; the scene is unchanged (no bullet visible — correct, none is spawned yet).
- [ ] Commit:

```bash
git add protos/AtlasBullet.proto worlds/ATLA_v1.wbt controllers/atlas_controller/tests/test_world_proto_layout.py
git commit -m "feat(world): add an importable AtlasBullet projectile proto"
```

---

## Task 3 — `atlas_bullet_controller`

**Model: sonnet (Codex: `gpt-5.4`) · Effort: low** — small, self-contained Webots controller; the structure is fully dictated below.

**Required:** Only the controller of the `Robot` that owns a `TouchSensor` can read it (Webots device ownership — see the spec's sensor discussion). The bullet therefore needs its own minimal controller to detect contact and publish the result where the turret supervisor can read it: the bullet's `customData` field.

**Webots controller-directory convention:** a controller named `atlas_bullet_controller` must live at `controllers/atlas_bullet_controller/atlas_bullet_controller.py` (directory name == entry-file basename == the `controller` field value), matching how `atlas_controller` is laid out.

**What to build — `controllers/atlas_bullet_controller/atlas_bullet_controller.py`:**

```python
"""ATLAS bullet controller — minimal collision reporter.

Runs on every dynamically-spawned AtlasBullet. One job: read the bumper
TouchSensor each step and, on the first contact, publish "HIT" to the
robot's customData field. The turret supervisor (atlas_controller) reads
that field to resolve the shot. See
docs/superpowers/specs/2026-05-18-turret-weapon-design.md.
"""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

touch = robot.getDevice("BULLET_TOUCH")
touch.enable(timestep)

# Latch: once the bullet has reported a hit, customData stays "HIT" for the
# rest of its (short) life — the supervisor removes the node promptly after.
reported = False

while robot.step(timestep) != -1:
    if not reported and touch.getValue() > 0.0:
        robot.setCustomData("HIT")
        reported = True
```

Notes for the implementer:
- A `bumper` `TouchSensor` returns `1.0` on contact and `0.0` otherwise — `> 0.0` is the contact test.
- The controller deliberately does **not** distinguish ground from ball; it only reports "something was hit." The turret supervisor disambiguates by the bullet's height (Task 5).

**Verification (manual, in Webots — combined with Task 4/5):** This controller cannot run standalone; it is exercised once the supervisor spawns a bullet. Confirm in Task 4's verification that no `atlas_bullet_controller` start-up error appears in the Webots console when the first bullet spawns.

- [ ] Commit:

```bash
git add controllers/atlas_bullet_controller/atlas_bullet_controller.py
git commit -m "feat(bullet): add the bullet collision-reporter controller"
```

---

## Task 4 — Spawn and launch the bullet

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** for the controller edit. **The verification/tuning step is orchestrator (Opus; Codex: `gpt-5.5`) or human** — `MUZZLE_SPEED` and `MUZZLE_OFFSET_M` are tuned by watching the bullet in Webots, not by haiku.

**Required:** When the FSM commits to a shot, the supervisor must put a bullet in the air along the aim direction.

**Current state of `controllers/atlas_controller/atlas_controller.py`:** A supervisor controller. `robot = Supervisor()`. The main loop calls `fsm.step()` at "4. Decide". `turret_position` is a fixed world-frame `[x, y, z]`. The FSM's intercept is turret-relative `[dx, dy, dz]`.

**What to change:**

1. Near the top-level constants (by `GROUND_HIT_THRESHOLD_M`), add:

```python
# --- Turret weapon constants ---
MUZZLE_SPEED = 18.0       # m/s — bullet launch speed (tune in Webots)
MUZZLE_OFFSET_M = 0.6     # m — spawn this far along the aim direction so the
                          #     bullet clears the turret's own bounding box
BULLET_GROUND_THRESHOLD_M = 0.05   # m — bullet world-Z at/below this = ground hit
BULLET_MAX_LIFETIME_STEPS = 400    # safety timeout: remove a bullet that never
                                   #                 reports (tune in Webots)
```

2. After the FSM is constructed (`fsm = AtlasFSM(...)`), get a handle to the scene root's `children` field for spawning, and initialise bullet bookkeeping:

```python
# Scene-tree root children — where dynamically-spawned bullets are inserted.
root_children = robot.getRoot().getField("children")

# Live-bullet bookkeeping. Only one bullet is alive at a time.
bullet_node = None         # the spawned AtlasBullet node, or None
bullet_age = 0             # timesteps since the live bullet was spawned
```

3. Add a `_fire_bullet(intercept)` helper above the main loop. `intercept` is turret-relative `[dx, dy, dz]`:

```python
def _fire_bullet(intercept):
    """Spawn an AtlasBullet at the muzzle and launch it toward the intercept.

    Args:
        intercept: turret-relative [dx, dy, dz] aim point (metres) from the
                   FSM fire command.
    Returns:
        The spawned AtlasBullet node.
    """
    # Unit vector from the turret toward the intercept (the launch direction).
    mag = math.sqrt(sum(c * c for c in intercept)) or 1.0
    direction = [c / mag for c in intercept]

    # Spawn position: world turret origin, offset along the aim direction so
    # the bullet starts clear of the turret's own bounding box.
    spawn = [
        turret_position[i] + direction[i] * MUZZLE_OFFSET_M for i in range(3)
    ]

    root_children.importMFNodeFromString(
        -1,
        'AtlasBullet {{ translation {:.5f} {:.5f} {:.5f} name "TURRET_BULLET" }}'.format(
            spawn[0], spawn[1], spawn[2]
        ),
    )
    node = root_children.getMFNode(-1)  # the node just appended

    # Launch: linear velocity along the aim direction, zero angular velocity.
    node.setVelocity(
        [direction[i] * MUZZLE_SPEED for i in range(3)] + [0.0, 0.0, 0.0]
    )
    log.info(
        "FIRE — bullet spawned at [%.2f, %.2f, %.2f], speed %.1f m/s",
        spawn[0], spawn[1], spawn[2], MUZZLE_SPEED,
    )
    return node
```

4. In the main loop, immediately after `fsm.step()` (step "4. Decide"), consume the fire command:

```python
    # 4b. Fire — spawn a bullet when the FSM commits to a shot.
    #     One bullet alive at a time: a fire command is ignored while a
    #     previous bullet is still in flight.
    fire_command = fsm.consume_fire_command()
    if fire_command is not None:
        if bullet_node is None:
            bullet_node = _fire_bullet(fire_command)
            bullet_age = 0
        else:
            log.info("FIRE ignored — a bullet is still in flight")
```

> **Note:** the existing `SimulatedProjectile` reset/relaunch block at the end of the loop (steps labelled "5. Manage projectile") is the incoming-ball logic and is being reworked separately. **Leave it untouched** — this plan neither depends on it nor modifies it. Task 5 adds the *bullet* lifecycle as a distinct block.

**Verification (manual, in Webots):**

- [ ] The controller starts with no exceptions (Webots console and `atlas_telemetry.log`).
- [ ] When the FSM reaches `ENGAGING`, a blue bullet appears at the turret and flies outward; the console logs `FIRE — bullet spawned ...`.
- [ ] No `atlas_bullet_controller` start-up error appears when the bullet spawns (this also verifies Task 3).
- [ ] **Tuning (orchestrator/human):** if the bullet collides with the turret on spawn, raise `MUZZLE_OFFSET_M`; if it visibly arcs far short of the ball, raise `MUZZLE_SPEED`. The bullet need not hit yet — Task 5 handles resolution. Re-test until the bullet launches cleanly along the aim line.
- [ ] Commit:

```bash
git add controllers/atlas_controller/atlas_controller.py
git commit -m "feat(controller): spawn and launch a bullet on the FSM fire command"
```

---

## Task 5 — Bullet lifecycle and hit resolution

**Model: sonnet (Codex: `gpt-5.4`) · Effort: medium** for the controller edit. **The safety-timeout value is tuned by orchestrator/human**, not haiku.

**Required:** A spawned bullet must be resolved and removed — otherwise bullets accumulate in the scene. The bullet self-reports contact via `customData`; the supervisor reads it, decides whether the ball or the ground was struck, and removes the node. A safety timeout covers a bullet that never reports.

**What to change in `controllers/atlas_controller/atlas_controller.py`:** Add a bullet-lifecycle block in the main loop, *after* the fire block (4b) and *before or after* the existing incoming-ball block — keep it as its own clearly-commented section:

```python
    # 4c. Manage the live bullet — resolve hits and remove the node.
    if bullet_node is not None:
        bullet_age += 1
        custom_data = bullet_node.getField("customData").getSFString()
        bullet_z = bullet_node.getPosition()[2]

        if custom_data == "HIT":
            # The bullet's TouchSensor fired. Disambiguate by height: a low
            # bullet hit the ground; a high one hit the ball.
            if bullet_z <= BULLET_GROUND_THRESHOLD_M:
                log.info("Bullet resolved — GROUND hit (miss)")
            else:
                # Hit signal for a future referee/scoring system to consume.
                log.info("Bullet resolved — BALL HIT at z=%.2f m", bullet_z)
            bullet_node.remove()
            bullet_node = None
        elif bullet_age >= BULLET_MAX_LIFETIME_STEPS:
            log.info(
                "Bullet resolved — lifetime timeout (%d steps), removing",
                bullet_age,
            )
            bullet_node.remove()
            bullet_node = None
        elif bullet_z <= BULLET_GROUND_THRESHOLD_M:
            # Fell to the ground without the sensor latching (e.g. a grazing
            # contact). Treat as a ground miss and clean up.
            log.info("Bullet resolved — reached ground without sensor latch")
            bullet_node.remove()
            bullet_node = None
```

Notes for the implementer:
- After `bullet_node.remove()` the Python handle is dead — always set `bullet_node = None` in the same branch so the next fire command can spawn a fresh bullet.
- The `customData` default for a freshly-spawned `Robot` is an empty string, so the `== "HIT"` test is safe before the bullet controller writes anything.
- "BALL HIT" is only logged here — scoring and the referee are explicitly out of scope (see the spec). The log line is the hand-off point for that future work.

**Verification (manual, in Webots):**

- [ ] Let the simulation run through a full engagement. When the bullet strikes the incoming ball, the console logs `Bullet resolved — BALL HIT ...` and the bullet disappears.
- [ ] When the bullet misses and falls, the console logs a `GROUND` resolution and the bullet disappears.
- [ ] Bullets never accumulate — at most one blue bullet is in the scene at any time.
- [ ] After a bullet resolves, the next `ENGAGING` spawns a new bullet (the one-bullet rule recovers correctly).
- [ ] **Tuning (orchestrator/human):** if a bullet is ever removed mid-flight by the timeout, raise `BULLET_MAX_LIFETIME_STEPS`; it should only ever trigger for a genuinely stuck bullet.
- [ ] Commit:

```bash
git add controllers/atlas_controller/atlas_controller.py
git commit -m "feat(controller): resolve and recycle the bullet on hit, ground, or timeout"
```

---

## Task 6 — ADR

**Model: haiku (Codex: `gpt-5.4-mini`) · Effort: low** — the ADR content is drafted below; mechanical authoring.

**Required:** Record the decision, consistent with the existing `docs/adr/` series (next free number is **0008**).

- [ ] Create `docs/adr/0008-turret-fired-projectile-weapon.md`:

```markdown
# ADR-0008: Turret Fired-Projectile Weapon

**Status:** Accepted

## Decision

The ATLAS turret destroys the incoming ball by firing a physical projectile
(the Bullet) along its aim direction, not with an instant-hit laser. The
Bullet is defined by `AtlasBullet.proto` as a `Robot` carrying a `bumper`
`TouchSensor` and a minimal controller (`atlas_bullet_controller`); it detects
its own collisions and publishes a hit through its `customData` field.

The FSM emits a one-shot fire command on entering `ENGAGING` and spawns
nothing itself. `atlas_controller`, already a Webots supervisor, consumes that
command, dynamically imports a Bullet at the turret muzzle via
`importMFNodeFromString`, launches it with `setVelocity()`, and owns the
Bullet's lifecycle (resolve on hit/ground/timeout, then remove the node).

The cosmetic `AtlasLaser` node is retained purely as an aiming line.

## Context

The system already has a `BallisticTrajectoryPredictor` and a `PREDICT` FSM
state that compute an intercept point ahead of the target. That pipeline only
earns its place if the weapon has travel time — an instant laser would aim at
the ball's current position and make the predictor dead code.

## Reasoning

**Fired projectile over laser:** Travel time forces the turret to *lead* the
target, making the predictor and intercept logic the core of the game rather
than vestigial code.

**Bullet is a `Robot`, not a `Solid`:** A `TouchSensor` can only be read by
the controller of the `Robot` that owns it. Detecting collisions with a real
sensor therefore requires the Bullet to be a `Robot` with its own controller.

**`customData` for the hit channel:** The Bullet controller publishes the hit
to `customData`; the turret supervisor reads that field. This avoids
`Emitter`/`Receiver` machinery for the first cut. Switching to an
`Emitter`/`Receiver` radio link is recorded as future work.

**FSM emits, controller spawns:** Node spawning is Webots-specific and not
unit-testable. Keeping it in the controller and having the FSM emit only a
plain fire command keeps the FSM pure and unit-tested.

**Dynamic spawn over a pre-placed node:** The turret supervisor imports a
Bullet only when it fires and removes it when the shot resolves — no stale
pre-placed node, no recycling of a parked node.

## Consequences

- New files: `protos/AtlasBullet.proto`, `controllers/atlas_bullet_controller/`.
  `worlds/ATLA_v1.wbt` gains an `IMPORTABLE EXTERNPROTO` declaration.
- The FSM gains a `fire_command` attribute and `consume_fire_command()`; it is
  additive and the existing `laser_active` flag is unchanged.
- `atlas_controller` owns bullet spawn, launch, and lifecycle, alongside the
  separate incoming-ball logic.
- Known limitation — tunnelling: a `bumper` `TouchSensor` can miss a fast
  bullet that passes through the ball within one timestep. Mitigated by
  capping `MUZZLE_SPEED` and sizing the Bullet generously; not solved.
- Future work: replace `customData` with an `Emitter`/`Receiver` link; a
  bullet pool for multiple shots in flight; tunnelling hardening.
- Scoring and the referee remain out of scope — the "BALL HIT" log line is the
  hand-off point for that future work.
```

- [ ] Commit:

```bash
git add docs/adr/0008-turret-fired-projectile-weapon.md
git commit -m "docs: record the turret fired-projectile weapon (ADR-0008)"
```

---

## Definition of done

- [ ] `cd controllers/atlas_controller && python -m pytest tests/ -v` — full suite green, including the new fire-command and `AtlasBullet.proto` layout tests.
- [ ] In Webots, when the FSM reaches `ENGAGING` a blue bullet spawns at the turret and launches along the aim direction.
- [ ] A bullet that strikes the ball logs `BALL HIT` and is removed; a bullet that reaches the ground logs a ground resolution and is removed; the safety timeout never fires in normal play.
- [ ] At most one bullet exists in the scene at a time, and engagements after the first still fire correctly.
- [ ] No existing `SimulatedProjectile` (incoming-ball) behaviour was modified.
- [ ] `AtlasLaser` is unchanged — still cosmetic.
