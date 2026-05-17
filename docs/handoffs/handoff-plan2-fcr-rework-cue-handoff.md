# Handoff — Plan 2: FCR Rework + Cue Handoff (issue #8, Increment 2)

## Focus for the next session

Execute **Plan 2** — `docs/superpowers/plans/2026-05-17-fcr-rework-and-cue-handoff.md`. This is the second and final increment of the radar-perception redesign (issue #8). Plan 1 (Physical Search Radar) is **done and merged** — see below.

## Where things stand

**Plan 1 is complete.** All 7 tasks implemented, each Opus-reviewed (spec + code quality), plus a final integration review. PR #10 (`Radar perception: design + Plan 1 (Physical Search Radar) — issue #8`) was approved and **the user is merging it to `main`**. Confirm it landed: `gh pr view 10` / `git log main`.

What Plan 1 produced (don't re-derive — read the code):
- `controllers/atlas_controller/geometry.py` — shared world-frame helpers `azimuth_elevation_range()`, `angle_diff()` (Z-up ENU). **Plan 2 reuses these.**
- `controllers/atlas_controller/search_radar.py` — `SearchRadar` with world pose, range/vertical-FOV gating, a rotating scan beam, and a `track_id`-keyed track buffer (ages/refreshes/expires on `track_timeout`). Output is still the turret-relative `Detection` NamedTuple.
- `protos/SearchRadar.proto` + `worlds/ATLA_v1.wbt` — a visible `Robot` node `DEF SEARCH_RADAR`.
- `controllers/atlas_controller/atlas_controller.py` — wired to read the `SEARCH_RADAR` node pose and construct `SearchRadar` with FOV/scan params.
- `docs/adr/0007-physical-search-radar.md`, updated `CONTEXT.md`.
- 140 unit tests pass (`cd controllers/atlas_controller && python -m pytest tests/ -v`).

## Two open items inherited from Plan 1

1. **Webots verification of Plan 1 was never run.** Plan 1's Definition of Done includes "the Webots sim runs and the FSM progresses past `SEARCH`" — this needs a live Webots run and was left to the user. Task 6's FOV/scan values (`max_range=20`, `vertical_fov=π/2`, `beam_width=0.35`, `scan_rate=0.15`, `track_timeout=20`) are **starting values** that may need tuning. If Plan 2 work surfaces that the FSM never leaves `SEARCH`, this tuning is the likely cause. The next session cannot run Webots either — flag it to the user.

## How to execute Plan 2

Same method as Plan 1: **subagent-driven execution**. An Opus orchestrator dispatches one fresh subagent per task, reviews between tasks (spec review, then code-quality review), then dispatches the next. Use the `superpowers:subagent-driven-development` skill.

Project-specific conventions established in the Plan 1 session — follow them:
- **Per-task model/effort tags.** Each Plan 2 task carries a `Model · Effort` tag; pass the tagged model to each `Agent` call. Orchestrator is always Opus; every between-task review is Opus. Webots parameter tuning is never delegated to haiku. A cheaper-model subagent that hits ambiguity must stop and escalate.
- **Use the `atlas-controls-engineer` agent type** for implementer and reviewer subagents (it knows this project's Webots/DSP/controls domain).
- **Announce the `model · effort` tag in visible text before every `Agent` dispatch** — the user monitors this. (Saved as agent memory `feedback_announce-subagent-model`.)
- **Parallelise where files are disjoint.** In Plan 1, tasks touching disjoint file sets were run in parallel. WARNING: parallel subagents each run `git add`/`git commit` — in Plan 1 one subagent's `git add` swept another's files into the wrong commit, and a doc task committed before a code task it described landed (stale docs). If parallelising: give each subagent a tight file scope, tell it to `git add` only its own files explicitly, and re-check commit boundaries afterward.
- Plans are written as execution strategy (requirements/approach/interfaces/verification), not literal code — a deliberate user preference (memory `feedback_plan-style`).

## Plan 2 scope (read the plan for the authoritative version)

`docs/superpowers/plans/2026-05-17-fcr-rework-and-cue-handoff.md` — 7 tasks, each tagged. Reworks the FCR into a grounded, FOV-gated slewing tracker cued by the Search Radar; the FSM's ACQUIRE/TRACK transitions reflect real FOV gating. Prerequisites in the plan list Plan 1 as done. Plan 2 Task 5 (world/proto) should inspect an existing proto and copy its conventions — `protos/AtlasTurret.proto` and `protos/SearchRadar.proto` are the references; `.wbt` wires project protos with plain `EXTERNPROTO "../protos/X.proto"` and instantiates `DEF NAME ProtoName {}`.

## Deliberately deferred (NOT in Plan 2 — do not let a subagent do these)

- `TrackFilter` migration from turret-relative to world frame. Each radar still converts to turret-relative internally.
- The Search Radar's own controller process + Emitter→Receiver radio link. The cue handoff in Plan 2 is an in-process value object crossing a narrow interface.

## First concrete steps

1. Confirm PR #10 merged to `main`; start Plan 2 work on a fresh branch off `main` (e.g. `feat/fcr-rework`) — confirm branch name with the user.
2. Read `docs/superpowers/plans/2026-05-17-fcr-rework-and-cue-handoff.md` and the design spec `docs/superpowers/specs/2026-05-17-radar-perception-design.md`.
3. Read the project memory: `C:\Users\madel\.claude\projects\C--Users-madel-source-repos-programming-robotics-project\memory\MEMORY.md`.
4. Invoke `superpowers:subagent-driven-development` and begin Plan 2 Task 1.

## Suggested skills for the next session

- `superpowers:subagent-driven-development` — to execute Plan 2 task-by-task.
- `superpowers:finishing-a-development-branch` — once all tasks are done.
