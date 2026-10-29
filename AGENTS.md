# AGENTS.md

This file governs how any agent (human or AI) works in this repository. Read it fully before doing anything. If a rule conflicts with a user instruction, stop and ask which one wins. Do not silently pick one.

---

## 1. What this project is

A **drop-in C++ core** for an Ackermann-constrained autonomous robot. The core is a blackbox with a single deterministic `step()` API. Everything outside the boundary — sensor drivers, hardware I/O, ROS, logging backend, clock, map loader, safety e-stop, localization — is unknown and out of scope. The core must compile and run without any of it via injected interfaces.

The core is designed to be dragged into another system and used as-is, without that system needing to know its internals.

---

## 2. Inputs, outputs, pipeline

### Inputs (per control cycle, immutable snapshot)

- Current pose `(x, y, θ)` in map frame (already localized).
- Goal `(x, y)`.
- LiDAR points `(r, α)` — the beam's length and current angle.
- Encoder RPM.
- Steering angle.
- Gyro yaw rate.
- Current global planner in effect (validated command, not raw per-cycle input).

### Output

`next_pose (x, y, θ)` — plus, at minimum, `dt`, `timestamp`, `frame_id`, and a `status` field. A pose-only adapter is allowed outside the core; the core itself must not collapse the output to bare pose.

### Pipeline (inside the core, deterministic order per `step()`)

`odometry → map manager (fixed map / SLAM / inflation) → global planner manager (A* | D*-Lite | RRT) → TEB local planner (Ackermann constraints) → output converter → status`

- **Odometry** — fuses encoder RPM, steering angle, gyro yaw rate into predicted motion.
- **Map manager** — exposes fixed borders, SLAM overlay, and inflation as replaceable layers, not as branches inside planners.
- **Global planner manager** — selectable via strategy/factory; produces a topological guide only; runs on demand (goal change, map change, timeout), not necessarily every cycle.
- **Local planner** — TEB with Ackermann constraints; runs every cycle; consumes pose, odometry, global guide, costmap, limits.
- **Output converter** — produces next pose (and status) at the defined control period.
- **Status** — explicit for valid, no-path, stale-sensor, planner-timeout, fault. No silent fallbacks.

---

## 3. Anti-hallucination rules (hard)

Non-negotiable. Violation is a bug.

1. **Never invent a value.** If a constant, dimension, timing, tunable, threshold, or unit is not in this file, not in a datasheet in `docs/`, or not in the user's message, you do not have it. Handle it via §6.
2. **Never guess a datasheet number.** If you "remember" a number, you don't have it. Mark it as missing and use §6.
3. **Every non-trivial claim must be sourced.** In code comments and reports: `[FROM: <source>]`, `[ASSUMED: <reason>]`, or `[UNKNOWN]`. No unlabeled facts.
4. **If two sources disagree**, stop and report both. Do not pick one.
5. **Never fabricate symbols.** If you don't know an exact API name, library function, file path, or ROS topic, say so and ask. Do not invent.
6. **No "typical" or "default" values smuggled in.** A default is a decision. Decisions are approved by the human, never inserted by the agent.
7. **If the task is ambiguous, stop and ask.** Do not resolve ambiguity by picking the interpretation that produces more code.
8. **No silent fallbacks in code.** If a required input is missing, the core must fail loudly (status code, guard, or `static_assert`) — never substitute zero.

---

## 4. Scope boundaries (hard)

Do only what the current task asks. Nothing else.

**You must not, unless the task explicitly says so:**

- Add, remove, rename, or restructure modules.
- Change the public header, `step()` signature, or the input/output contract in §2.
- Add third-party dependencies (Eigen, Boost, ROS, OpenCV, etc.) — ask first.
- Introduce threads, async execution, or hidden globals.
- Add file I/O, network I/O, logging backends, or hardware SDK calls inside the core.
- Refactor "while you're in there". No drive-by improvements.
- Change frame names, unit conventions, or sign conventions in §5.
- Convert a placeholder (§6) into a real value. That is a human decision.
- Write tests for things not in the task.
- Update this file unless the task is specifically "update AGENTS.md".

**You may:**

- Read any file in the repo.
- Propose changes (as a plan, §8) before making them.
- Add TODO markers that reference a placeholder ID (§6).
- Ask clarifying questions at any point.

If the task appears to require crossing a boundary, stop and state: _"This task appears to require X, which is outside the current scope. Confirm before I proceed."_

---

## 5. Conventions

- **Language**: C++ (assume C++17 unless told otherwise).
- **Standard Library First**: Use standard C++ library algorithms and containers (e.g., `<algorithm>`, `<queue>`, `std::priority_queue`, `std::vector`) whenever a standard implementation exists. Do not hand-roll basic data structures (like custom heaps, sorting algorithms, or dynamic arrays) unless explicitly instructed or proven strictly necessary.
- **Allocation Policy**: While §5 forbids hidden state and wall-clock reads, standard library allocations (like `std::priority_queue` or `std::vector`) are permitted during planner setup or execution unless a strict zero-allocation constraint is explicitly requested. Prioritize code correctness and standard algorithms over avoiding standard library heap allocations.
- **Frames**: `map`, `odom`, `base_link`, `lidar`, `imu`. No others without approval.
- **`base_link` origin**: the center of the rear axle on the ground plane. +x points forward, +y points left. Sensor mount offsets (for example `LIDAR_MOUNT_X_M`) are measured from this origin. The TEB kinematic model (`tan(δ)/L`) depends on this reference point.
- **Units**: meters, radians, seconds, rad/s. No degrees or milliseconds in the core API.
- **Angles**: θ normalized to `(-π, π]` at the boundary.
- **Signs**: positive steering = left turn, positive yaw = CCW, unless a placeholder (§6) states otherwise for a specific sensor.
- **No globals, no singletons, no hidden state** in `step()`.
- **Deterministic**: same input snapshot → same output. No wall-clock reads inside `step()`.
- **Public boundary**: one header, one facade, one `step(const InputSnapshot&) -> Output`.
- **Pose-only adapter** lives outside the core, not inside `step()`.

---

## 6. Handling unknown data + the unknowns registry

Any value, parameter, constant, threshold, calibration, transform, or configuration that is not given to you is **unknown**. Never invent it, never assume a "typical" value, never proceed silently.

### 6.1 Placeholders in code

For every unknown, all of the following apply:

1. **Define it as a compile-time placeholder** using `#define` (or an equivalent named constant in a dedicated placeholder header). Use a greppable naming convention so placeholders are easy to find across the repo.
2. **Use a sentinel value that cannot be valid in the real system** (zero for dimensions, angles, times; a dedicated enum value for policy choices; an explicit null-like state for object references). Zero must never be a "usable default" for any physical quantity.
3. **Add a runtime guard** — in `initialize()` or equivalent — that refuses to operate if a placeholder affecting motion has not been replaced. The guard must be loud (status code, refused init, or `static_assert`), never silent.
4. **Tag the placeholder** in code with `[UNKNOWN]` and a one-line note: what it is, what units, what it affects, what the guard does.
5. **Do not fill it in.** Replacing a placeholder with a real value requires explicit human approval in the task. If you believe you know the value, propose it in Phase A, do not write it in Phase B without approval.

What is a placeholder vs. runtime config:

- **Compile-time placeholder** (`#define`): physical invariants, transforms, calibration, limits, frame names, sensor configuration. Things that would change the math if wrong.
- **Runtime config struct**: tunables the human expects to adjust at runtime (planner weights, thresholds, map parameters). Still no invented values — the config must be supplied by the caller.

### 6.2 The unknowns registry — `UNKNOWNS.md`

A single living file at the repo root, `UNKNOWNS.md`, is the **authoritative list of every unknown variable currently in the codebase**. It is the only place unknowns are listed. This file (`AGENTS.md`) defines how the registry is maintained; it does not enumerate specific unknowns.

**Rules:**

- Every `#define` placeholder created in code **must** have a row in `UNKNOWNS.md` in the same change.
- No placeholder without a registry row. No registry row without a code placeholder (or a proposed one in the current plan).
- Rows are never deleted. When a value is supplied, mark the row `RESOLVED` and note where it was replaced and how the guard was removed. If a placeholder is removed without a value (e.g., the code was dropped), mark it `DROPPED`.
- The registry is append-only for history; git carries the audit trail.

**Row format** (markdown table, one row per unknown):

| Field         | Meaning                                                                                                                                   |
| ------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `ID`          | Stable short ID, e.g. `PH-0001`, never reused                                                                                             |
| `Placeholder` | Exact `#define` name used in code                                                                                                         |
| `Type / Unit` | e.g. `double / m`, `enum`, `Pose2D`                                                                                                       |
| `Sentinel`    | The sentinel value used, and why it cannot be valid                                                                                       |
| `Affects`     | Which pipeline stage(s) or module(s)                                                                                                      |
| `Guard`       | What guard fires if the placeholder is unreplaced                                                                                         |
| `Added by`    | Task/commit that introduced it                                                                                                            |
| `Status`      | `PROPOSED` (in current plan, not yet in code), `OPEN` (in code, guard in place), `RESOLVED` (replaced), `DROPPED` (removed without value) |
| `Resolution`  | For `RESOLVED` or `DROPPED` only: what happened, by whom, when                                                                            |

### 6.3 When the registry is updated

- **Phase A**: any new unknown proposed in the plan gets a `PROPOSED` row written into the plan, not yet into `UNKNOWNS.md`.
- **Phase B**: on approval, when the placeholder is written into code, its row is appended to `UNKNOWNS.md` in the same change (same commit / same step). Status becomes `OPEN`.
- **Resolution**: when the human supplies a value, the change replaces the `#define`, removes the guard, and flips the row to `RESOLVED` with a resolution note. This is a normal task, not implicit cleanup.

If `UNKNOWNS.md` doesn't exist yet when the first placeholder is added, create it with a short header describing its purpose and the row format above.

---

## 7. Markdown conventions for all agent output

All agent-facing reports (Phase A plans, Phase B progress notes, Phase C reports, questions) are **well-structured markdown**. The goal is skimmability — the human should be able to scan and verify without reading every sentence.

**Structure:**

- Use `##` and `###` headings to separate logical sections (one section per Phase A item, one per Phase C item, etc.).
- Use tables for: unknowns (per §6), files to touch, step lists, verification matrices, assumptions. Consistent column headers.
- Use task lists (`- [ ]` / `- [x]`) for execution steps in Phase B; check each off as it completes.
- Use fenced code blocks for file paths, identifiers, commands, snippets, and the `UNKNOWNS.md` row template.
- Use inline code for symbol names, file names, frame names, placeholder IDs (`PH-0001`), statuses (`OPEN`).
- Use callouts — `> **Stop:** …`, `> **Note:** …`, `> **Assumption:** …` — for anything that would halt execution or needs the human's attention.
- One action per numbered list item. If an item has two verbs, split it.
- No walls of prose. If a paragraph exceeds ~5 lines, restructure it as a list or a table.
- No empty tables. If a section has nothing to report, write "None" — do not include a stub table.

**Phase A output, in particular, follows these conventions:** each of the eight required sections (§8, Phase A) is its own `###` heading, and wherever the content is naturally a list (unknowns, files, steps, verification) it is rendered as a table or a task list, not prose.

---

## 8. Workflow (mandatory, every task)

Every task runs in three phases. Do not skip or merge phases. All output uses the markdown conventions in §7.

### Phase A — Plan (produce, do not execute)

Output a markdown plan with these `###` sections, in this order:

1. **### Restatement** — one paragraph: what the task asks, in your own words.
2. **### Assumptions** — table of every assumption: `#`, assumption, tag (`[ASSUMED: reason]`). If none: "None."
3. **### Unknowns** — table of every fact you need but don't have, with the columns from §6.2 plus a `Sentinel` and `Guard` column. Status is `PROPOSED`. If none: "None."
4. **### Files to touch** — table: path, change summary, reason. If a path doesn't exist yet and isn't in the pipeline (§2), flag it in the row.
5. **### Steps** — numbered task list (`- [ ]`), one action per item, each with what it changes and why.
6. **### Out of scope** — explicit list of what you are _not_ doing, to prove you didn't overreach.
7. **### Verification** — table: step #, how it's checked (compile, guard fires, status code, etc.).
8. **### Stop conditions** — callouts (`> **Stop:** …`) for anything that would make you halt and ask.

Then **stop and wait for approval.** Do not write code in Phase A. Do not create or edit `UNKNOWNS.md` yet — proposed rows live in the plan only.

### Phase B — Execute

- Follow the approved plan step by step, in order. Track progress as a task list.
- After each step, note: file changed, what changed, verification result.
- When a placeholder step executes, append its row to `UNKNOWNS.md` in the same step.
- If reality diverges from the plan (missing symbol, wrong assumption, guard fires unexpectedly), **stop**, report the divergence as a callout, and return to Phase A with a revised plan. Do not improvise.
- No extra steps. No "since I was here, I also…".

### Phase C — Report

Markdown report with these `###` sections:

1. **### What changed** — table: file, one-line summary.
2. **### Plan deviations** — table or "None." If any, each row needs justification.
3. **### New placeholders** — table matching `UNKNOWNS.md` columns for any `[UNKNOWN]` introduced in this task, with status `OPEN`.
4. **### Registry update** — confirm `UNKNOWNS.md` was updated (yes/no) and list the IDs added or resolved.
5. **### New assumptions** — table tagged `[ASSUMED]`, or "None."
6. **### Not done / blocked** — table: item, reason, proposed next step, or "None."
7. **### Verification results** — table: check, result.
8. **### Open questions** — numbered list for the human, or "None."

---

## 9. What to do when stuck

1. Stop.
2. State exactly what is missing or contradictory, as a `> **Stop:** …` callout.
3. Propose the smallest question that unblocks you.
4. Wait.

Never proceed on a guess to "keep momentum". Moving fast on a guess is the failure mode this file exists to prevent.
