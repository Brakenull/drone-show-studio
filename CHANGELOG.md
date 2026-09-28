# Changelog

All notable changes to Drone Show Studio are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- **Blender add-on — holding area in the scene:** a **Create in Scene** toggle in the Holding Area box builds the takeoff area as real objects: a wire box for the volume and a sphere per launch slot for the whole fleet, placed where the drones will really be (heading offset included). Grab the box to move the holding area; the panel settings follow. The panel also shows the slot grid and warns when the area had to be widened.
- **Blender add-on — show clearance caution:** a **Safe Distance to Show** setting (default 5 m) in the Holding Area box. Check Kinematics, Auto-Fix and Export now check every formation against the holding area and show a caution, per formation, for points inside it or closer than the safe distance. **Export is locked** while any caution stands, like a kinematic error. With **Create in Scene** on, a yellow wire box shows the safe-distance zone.

- **Blender add-on — takeoff and return legs:** a **Takeoff & Return** box sets each leg to **Auto** (as fast as safe) or a **target time**. The panel estimates each leg's minimum time (the longest flight at top speed, with the same margin as Auto-Fix, plus the row-by-row takeoff waves), warns when a target is below it, and sums up takeoff + show + return. The export carries the targets in a new optional `legs` field (schema **1.6.0**; 1.5.0 files stay valid).
- **Path planning — takeoff and return legs:** for files with `legs`, Stage 2 flies the takeoff in max(target, minimum time), or the minimum for Auto. It starts the show's timeline when the takeoff reaches the first formation, and adds a **return leg** that lands every drone at rest on a free holding-area slot with its LEDs fading to off, checked by the same 100-per-second safety check. The output reports each leg's actual start, end and duration (`metadata.legs`), and `total_duration_sec` includes the return. Files without `legs` plan exactly as before. The desktop app names the return leg "to the holding area".

- **Blender add-on — ground level:** a **Ground** box sets the ground height (default z = 0). The panel warns about formation points and parked drones below it, and **Export is locked** until nothing is below the ground. **Show Ground in Scene** draws a wire grid at that height under the show and the holding area. The ground is exported as `ground_z_m` (schema 1.6.0), and the desktop app's replay uses it for its "below ground" list instead of a fixed z = 0.

- **Path planning — planned vs flown time:** the output lists every transition with its start and end show time, planned and flown duration, and the number of attempts (`metadata.transitions`).

### Fixed

- **Path planning — paths below the ground (P2-02):** Stage 2 now uses the design's ground level (`ground_z_m`) as an altitude floor. No point of any planned path goes below it, including takeoff, formations passed near the ground and the return. A show whose formations or holding area are below the ground is refused before planning. Files that declare no ground are planned as before.
- **Path planning — timing after a safety retry (P2-03):** a transition that passed only on a retry was flown longer than the timeline said. The next transition then started while it was still being flown, LED fades overshot, and the show's reported length (and the desktop app's replay) ended early. Every transition is now timed by the attempt that actually passed.

### Changed

- The holding area now counts as its whole declared volume plus the parked grid (padded by half a grid step), in both the add-on and the desktop app's "formation overlaps the holding area" warning. Before, the app only counted the parked grid, which a small fleet fills only partly.

## [1.0.0] — 2026-09-26

First release. Drone Show Studio takes a drone light show from a Blender animation to verified, per-drone flight files, and is distributed as source code with a one-step setup.

### Added

#### Show design — Blender add-on (`stage1_designer`, add-on version 1.5.0)

- Turns animated meshes into exactly one point per drone: Poisson-disk sampling on surfaces, jittered-grid sampling inside solids, always at least 1.5 m apart.
- Checks the speed needed between keyframes against the drone's top speed (OK / warning / error), and **Auto-Fix** stretches the timeline where a move is too fast.
- Lays out the takeoff holding area on a 2 m grid, stacking layers and widening the area when the fleet doesn't fit.
- Converts to East-North-Up coordinates with a heading offset, and shows a True-North compass in the viewport.
- Draws a safety sphere around every drone in the viewport, blinking red where two are too close.
- Exports the show as JSON or MessagePack (schema version 1.5.0), refusing to export a show with the wrong point count, duplicate drone numbers or points closer than the minimum distance.

#### Path planning — core engine (`stage2_core_engine`, the `drone_core` Python module)

- Assigns every drone to a point in the next formation with the Auction algorithm, weighing distance, climbing and turning.
- Plans each move as a smooth quintic B-spline within the speed, acceleration and jerk limits, stretching a transition that is too short to fly.
- Bends paths apart to avoid collisions (sequential convex programming with the OSQP solver), with parallel solving on all CPU cores.
- **Independent safety check:** every pair of drones is re-checked 100 times per second. If a transition still brings two drones closer than 1.45 m after two retries, the show is rejected — never returned. The rejection names the transition, every pair that was too close, when and where.
- Optional staggered takeoff by rows, re-checked and replaced by a synchronized departure when staggering would be less safe.
- Progress events while planning, and the solve can run for hours on large shows without growing memory.
- Planner settings in `config/core_config.json`, overridable per run.

#### Digital twin and flight files (`stage3_simulation_packer`)

- Simulates every drone as a quadcopter with its own controller, motors, battery (voltage sag, temperature, Peukert effect) and the downwash of drones flying above it, using NVIDIA Warp on the CPU or an NVIDIA GPU.
- **Stress test:** flies the show under many random weather draws — mean wind up to 8 m/s, turbulence, a gust front up to 5 m/s, positioning (RTK) drift, per-drone hardware spread. It passes when no two drones ever come within 0.5 m and every drone lands with at least 15 % battery; closer than 1.0 m is reported as a close call. Runs are reproducible from their seed and can use several processes.
- Drone hardware described in `config/drone_profile.json`; unknown keys are rejected.
- **Flight files:** one `drone_<id>.bin` per drone with position, velocity and LED colour every 50 ms, a CRC-32 checksum and a `manifest.json`. Every file is read back and verified after writing; a coordinate out of range stops the packer instead of being clipped.
- Reads the planner output as JSON, Apache Arrow, or shared memory.

#### Drone Show Studio desktop app (`apps/`)

- **New run:** drop a show file on the window (or pick it) to validate it. Every error is listed with its location, plus warnings for problems path planning can't fix — for example a formation that overlaps the holding area — and the holding area's capacity.
- **Stage 2:** plans the show in the background with live progress and Cancel. A rejected show lists the too-close pairs, each linking to the replay at that moment.
- **Planner settings:** every setting with its default and help text; a change that weakens safety must be confirmed before running and stays visible on the result. "Try these settings in a new run" keeps the original for comparison.
- **Replay:** 3D view of every drone with its LED colours, a scrubber that doubles as a chart of the closest distance between any two drones, and highlighting of any pair.
- **Stage 3:** runs the stress test with a live light for each flight and a results table, then packs and verifies the flight files.
- **Compare:** two runs' closest distance over time, their results side by side, and the settings that differ.
- Every run is kept in its own folder under `runs/` and reopens without recomputing.
- Dark design based on the SkySync design system, with the Inter and Fira Code fonts bundled.

#### Setup

- `setup.ps1`: checks the prerequisites (and lists what is missing), creates the Python environment, builds and tests both engines, checks the whole pipeline with a test show, and installs the app's packages.
- Pinned dependencies: `requirements.txt` / `requirements-dev.txt` for Python, `vcpkg.json` for the C++ libraries (installed automatically by CMake), and CMake presets for building by hand.
- `README.md` with setup, usage, command-line use and troubleshooting.

### Verified

- 158 automated Python tests and 4 C/C++ unit test programs pass; the Stage 2 smoke test passes.
- A 100-drone and a 150-drone show plan successfully; a 20-flight stress test of the 100-drone show passes (closest approach 1.08 m, lowest battery at landing 88 %).
- Flight files and the stress-test report from the app match the command-line tools exactly.

### Known issues

- **A transition that passes only after a retry** is flown longer than the show's timeline says. The show length, the replay and the colour fades are then off by the extra time; if it isn't the last transition, it overlaps the next one, and the safety check hasn't checked that overlap. Check the Stage 2 output for retried transitions until this is fixed.
- **No minimum altitude:** path planning can bend paths below the ground (z = 0) to keep drones apart. The replay lists every drone that goes below ground; check it before flying.
- **A safety distance set above about 3 m is not fully checked.** The app warns about such a setting; the default of 1.45 m is not affected.
- **Formations overlapping the holding area** usually can't be planned safely. The Blender add-on doesn't check this yet; the app warns when a show file is validated.
- **GPU speed is unverified:** the target of simulating 1 000 drones in real time on an RTX 3060 or better has not been measured. On a CPU, 1 000 drones run at about half real time.
- The replay samples 20 frames per second, so its closest distance can differ from the 100 Hz safety check by a few millimetres.
- Windows 10/11 x64 only. The installer made by `npm run tauri build` contains only the app window; computation still needs this repository set up on the same machine.

### Requirements

Visual Studio 2022 or later with "Desktop development with C++", Python 3.14 (64-bit), Node.js 20 or later, Rust, Git. Blender 4.0 or later for show design. An NVIDIA GPU is optional.

[1.0.0]: https://github.com/Brakenull/drone-show-studio/releases/tag/v1.0.0
