# Integrating `robot_core` on the lidcar ESP32: fit and gap review

**Reviewed:** `Mohamm-a/path-planning-algorithms`, branch `build`.
**Target:** lidcar firmware on the ESP32-S3 DevKitC-1 (Zephyr 4.4, C++20, 240 MHz Xtensa LX7).
**Status:** discussion draft. Figures marked *estimate* have not been measured on the target.

This review asks how `core::Core` can run on the car, and what we need from each other to get
there. It is not a review of the planning method itself.

## 1. Summary

`robot_core` is a full planning pipeline:

```
odometry -> map manager (fixed / SLAM / inflation) -> global planner (A*) -> TEB local planner -> output
```

The interface is clean and well suited to embedding. All buffers are owned by the caller, it uses
no globals and no threads, and its status codes are explicit. Running it on the ESP32 raises three
topics:

| # | Topic          | Today                                        | On ESP32-S3                                              |
|---|----------------|----------------------------------------------|----------------------------------------------------------|
| 1 | C++ library    | `std::vector`, `std::make_unique`, `<cmath>` | Needs the full C++ library and a heap; we prefer no allocation in `step()` |
| 2 | Memory         | Planner workspace ≈ 21 B per cell (32-bit)   | Fits in our 8 MB PSRAM, but PSRAM is slower than SRAM    |
| 3 | Floating point | `double` throughout                          | The FPU is single precision; `double` is done in software |

Topic 3 is the main risk, because it decides whether a step fits in the 10 Hz loop. None of these
are design flaws. They are portability choices that we should agree on together.

## 2. Interface fit

The lidcar framework will wrap `core::Core` in an adapter. We write and maintain the adapter, so the
algorithm code does not have to change to match our conventions.

| Item          | `robot_core`                         | lidcar framework                       | Adapter action                  |
|---------------|--------------------------------------|----------------------------------------|---------------------------------|
| Units         | m, rad, s (`double`)                 | cm, centi-degrees, ms (integer)        | Convert                         |
| Body frame    | +x forward, +y left, origin at rear axle | +y forward, +x right                | Rotate and mirror               |
| Lidar angle   | α counter-clockwise from lidar +x    | 0 = right, 90 = ahead, 180 = left      | α = (angle − 90)° in rad        |
| Lidar data    | full scan every cycle (in the sim)   | 180-point sweep about every **5 s**    | See question Q3                 |
| Pose          | supplied by the caller (localized)   | dead reckoning from encoder + gyro     | We supply odometry pose only    |
| Goal          | required                             | not defined yet                        | See question Q4                 |
| Output        | `next_pose` (x, y, θ)                | speed + heading setpoint               | Convert, or see question Q5     |
| Cycle         | `dt` given by the caller             | 10 Hz navigation thread                | Direct                          |

## 3. Placeholders we can supply

We can provide most of the 17 values in `placeholders.hpp` from the car hardware. We will send
measured values separately.

| Placeholder                         | Source                                           |
|-------------------------------------|--------------------------------------------------|
| `WHEELBASE_M`, `WHEEL_RADIUS_M`     | Measured on the chassis                          |
| `MAX_STEER_RAD`, `MAX_STEER_RATE_RADPS` | MG996R steering servo limits and linkage     |
| `MAX_SPEED_MPS`, `MAX_ACCEL_MPS2`   | Motor + L298N, measured with the encoder         |
| `LIDAR_MOUNT_X_M/Y_M/THETA_RAD`     | Measured; TF-Luna on the scan servo              |
| `LIDAR_MAX_RANGE_M`                 | TF-Luna datasheet (8 m), maybe derated indoors   |
| `ROBOT_RADIUS_M`                    | Chassis size                                     |
| `SENSOR_STALE_TIMEOUT_S`            | Must allow for a 5 s sweep (see Q3)              |
| `MAP_RESOLUTION_M`, `INFLATION_RADIUS_M`, `GLOBAL_PLANNER_REPLAN_PERIOD_S`, `PATH_SMOOTH_*` | Tuning; your proposal, checked against memory |

## 4. Target constraints

### 4.1 Memory

The README gives the workspace size as N × (8 + 3 × sizeof(size_t) + 1) bytes, for N grid cells.
On a 32-bit target this is 21 B per cell, and on a 64-bit host 33 B per cell. There are also four
1-byte grid layers.

| Area     | Cell size | Cells   | Workspace (32-bit) | Grids  | Total    |
|----------|-----------|---------|--------------------|--------|----------|
| 10 × 10 m | 5 cm     | 40 000  | 840 KB             | 160 KB | ≈ 1.0 MB |
| 10 × 10 m | 10 cm    | 10 000  | 210 KB             | 40 KB  | ≈ 250 KB |
| 5 × 5 m  | 10 cm     | 2 500   | 53 KB              | 10 KB  | ≈ 63 KB  |

The ESP32-S3 has about 512 KB of internal SRAM. The BLE stack, the other threads and the stacks use
part of it. Our board also has **8 MB of embedded octal PSRAM** (confirmed with `esptool`: "Embedded
PSRAM 8MB"). Zephyr supports it (`CONFIG_ESP_SPIRAM`, `CONFIG_SPIRAM_MODE_OCT`), and static buffers
can be placed there through the `.ext_ram.bss` linker section.

So memory size is **not** a blocker. Even a 10 × 10 m map at 5 cm (about 1 MB) fits. The
trade-offs are:

- PSRAM is slower than internal SRAM. It is accessed through a cache, so random access over large
  grids (A* expansion) costs more than on a PC. This adds to the CPU-time concern in 4.3.
- We would keep the grids and the planner workspace in PSRAM, and keep the hot TEB state (small)
  in internal SRAM.
- The octal PSRAM uses GPIO33–37. Our pin plan already avoids these pins.

### 4.2 Heap and standard library

Zephyr can provide the full GNU C++ library and a heap, so `std::vector` and `std::unique_ptr` are
possible. However, allocation inside `step()` (for example the `std::vector` objects in
`TebLocalPlanner::solve`) causes heap fragmentation and makes timing unpredictable over long runs.
We would prefer:

- all allocation in `initialize()`, or in caller-owned buffers as the grids already are;
- no allocation in `step()`.

### 4.3 CPU time

On a desktop PC (x86-64, `-O2`), the wall simulation took about **12 ms per cycle**, using throwaway
placeholder values. The ESP32-S3 runs at 240 MHz and does `double` arithmetic in software. Our
*estimate* is **0.5–2 s per cycle** as the code is today. The 10 Hz loop allows 100 ms, and we
would like to keep the navigation step at or below about 20–50 ms.

The biggest wins are likely to be:

- `float` instead of `double`;
- fewer TEB iterations or a shorter horizon (the sim uses 100 iterations and 10 steps, with
  finite-difference gradients);
- running the global planner less often than the local planner (`GLOBAL_PLANNER_REPLAN_PERIOD_S`).

## 5. How the code is shared

`robot_core` is a **west project** in the lidcar manifest (`west.yml`). It is not copied into the
lidcar repository.

```yaml
- name: path-planning-algorithms
  remote: algo-team                 # https://github.com/Mohamm-a
  revision: 43025eb124bf...         # a pinned commit on the 'build' branch
  path: algorithms/robot_core
```

`west update` checks it out next to Zephyr. This gives the following rules:

| Rule | Meaning |
|------|---------|
| Your repository, your code | The lidcar team never edits files under `algorithms/robot_core`. Fixes and requests go to you as issues or pull requests. |
| Pinned versions | The car always builds against an exact commit. Moving to a newer one is a one-line, reviewed change to `west.yml`. Please tag or name commits you consider ready. |
| No Zephyr files needed | The lidcar build compiles your `src/*.cpp` itself and supplies the placeholder values with `-D` flags. You do not need a `zephyr/module.yml`, Kconfig or Zephyr CMake. |
| Host build stays yours | Your `CMakeLists.txt`, tests and `wall_sim` keep working unchanged. |
| Access | Everyone who builds the car needs read access to the repository. Please tell us before making it private. |

The adapter that connects `core::Core` to the car lives in the lidcar repository (`src/nav/`). It
converts units and axes, feeds sensor data and turns `Output` into speed and steering commands.
Only the adapter depends on both code bases.

## 6. Checking a `float` conversion

Changing `double` to `float` (Q1) can change results in subtle ways. Examples are angle wrapping
near ±π, small finite-difference steps (`fd_epsilon`), and costs summed over many cells. We propose a
regression check that both teams can run on a PC:

1. Build `wall_sim` twice from the same commit, once with `double` and once with `float`. A single
   `using real = ...;` alias makes this a one-flag change.
2. Use the same agreed placeholder values for both builds.
3. Run both, and compare the per-cycle output (`x`, `y`, `theta`, `status`).
4. Pass criteria, to be agreed (*proposal*): the same status every cycle, the same `RESULT`, and a
   position difference below one map cell throughout the run.

Several scenarios (the wall, open space, a narrow passage) would make the check stronger than the
single wall run. The same runs also give a host baseline for step time.

## 7. Questions for the algorithm team

| #  | Question |
|----|----------|
| Q1 | Can the core use `float`, perhaps through a single `using real = float;` type alias? |
| Q2 | Can `step()` run without heap allocation, using preallocated or caller-owned buffers? |
| Q3 | A lidar sweep takes about 5 s. Should we pass the latest complete sweep every cycle with its original timestamp, or only once when it arrives? How should the SLAM layer and the stale check treat it? |
| Q4 | Where does the goal come from in your use case: a fixed point, a BLE command, or "explore"? Is there a mode with no goal, for obstacle avoidance only? |
| Q5 | Could `Output` also expose the first-step commands `v_cmd` and `delta_cmd`, which TEB already computes? This would avoid converting `next_pose` back into commands. |
| Q6 | What map size and cell size do you need, at minimum? |
| Q7 | What `step()` time can you accept, and which TEB settings (iterations, horizon) are you willing to reduce? |
| Q8 | Are odometry-only poses (encoder + gyro, with drift) acceptable, or do you expect a localization source? |

| Q9 | Do you agree with the sharing rules in section 5 and the `float` check in section 6? |

## 8. Proposed next steps

1. Agree on Q1–Q9.
2. Build `robot_core` unchanged for the ESP32-S3, with the full C++ library, a heap, and the grids
   in PSRAM, and measure the real `step()` time and memory use. The lidcar team owns this step.
3. Agree on any changes to the core, which the algorithm team makes.
4. The lidcar team writes the framework adapter and runs a first drive with measured placeholder
   values.
