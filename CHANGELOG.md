# Changelog

All notable changes to Drone Show Studio are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow [Semantic Versioning](https://semver.org/).

## Release notes (1.2.1 — 2026-10-05)

**Rain return planning now tells you how to close the gaps, and the flights home travel with the show.**

- **Suggestions for uncovered moments.** When a rain window doesn't cover the whole show, the readiness panel lists what would help, most useful first: plan returns from inside a long move, start the return earlier, plan a missing return path, re-plan a slow return, or move a formation closer to the holding area. Each shows how much of the gap it closes, and one click acts on it.
- **Return paths from inside a move.** The fleet no longer has to finish a long move before turning for home. On `200_cube`, one such return cut the rain window the show needs from 126 s to 97 s, and it flew home in the digital twin 29 s ahead of the rain limit.
- **Flights home in the flight files.** Each drone's file now carries its planned flights home and a table saying which one to take when the return is called. This is a new file format (version 2).
- **Fixes:** planning from a suggestion now shows its progress where you clicked, and weather values beside the timeline can be cleared while typing.

**Before you upgrade:** anything that reads flight files must handle version 2. No drone firmware does yet, and the command that tells the fleet to return isn't built, so the flights home can be planned, simulated and packed but not flown by real drones.

## [Unreleased]

## [1.2.1] — 2026-10-05

The rain return readiness panel now says what would close the time a rain window doesn't cover, and the flights home it plans travel with the show in the flight files. Return paths can be planned from moments inside a move, not only from formations, so the fleet no longer has to finish a long move before turning for home: on `200_cube` that alone brings the rain window the show needs from 126 s down to 97 s.

### Added

#### Suggestions for uncovered moments (desktop app, `twin_sim`)

- **What would close the gap:** under the readiness chart, "Suggest for *W* s" lists the options for that rain window, most useful first, each with a bar showing how much of the uncovered time it closes on its own and its numbers. The options:
  - **Plan returns from inside a move:** candidate moments every 10 s inside the move whose time home is too long, with an estimated flight home. Only those that help are kept.
  - **Start the return earlier:** how many seconds before the rain alert, and the lower alert level that would give that time with the rain rising steadily. When no alert level can, it says a forecast is needed.
  - **Plan a return path** from a formation that has none.
  - **Plan a return again at its minimum time**, offered only when it would save time. Otherwise it says the return is already as fast as the motion limits allow and names the farthest drone.
  - **Bring a formation closer to the holding area:** how much shorter its return would need to be (a change in Blender).
- **One click to act:** "Plan it / Plan them", "Plan again" or "Use *x* mm/h" (sets the scenario's alert level as an unsaved edit). The planning progress, with Cancel, shows in the row you clicked. The list dims and asks to suggest again when the window, the rain rule or the return paths change.
- Command line: `studio_bridge suggest <run> --window-s W`. Estimates take well under a second and are cached per Stage 2 result.

#### Return paths from inside a move (path planning, desktop app)

- **Abort points:** a return can now start at any moment strictly inside a move, from the drones' positions, speeds and accelerations at that moment, and is checked like every other return. When the return is called during that move, the fleet flies on to whichever planned way home ahead of it gets it home first, an abort point or the formation, and turns for home there.
- **In the app:** planned from the suggestions, then listed under Stage 2 › Return paths › "From inside a move" (the move, the moment, the flight home, closest approach, status, View and Plan again). The replay is labelled "Flight home from the move to *formation* at *time*".
- **In the simulator:** a scenario whose rain calls the fleet home before an abort point flies on to it and returns from there; the result and the heads-up display say so.
- Command line: `studio_bridge stage2-returns <run> --points K@T,...`. Python: `drone_core.plan_return_path(..., abort_time_sec=...)`, and `drone_core.estimate_return_paths()` for the minimum flight time of many starts without planning them.

#### Flights home in the flight files (Stage 3)

- **Flight files version 2:** each drone's file now carries the show and its planned flights home as separate tracks, plus a return table that says which flight home to take when the return is called at any moment, and a pack id shared by every file of one pack. Only the flights home the table uses are packed. "Pack flight files" builds the table from the same planning as the readiness chart. The Flight files card lists the flights home packed and warns when return paths were planned after packing. Command line: `pack_to_binary --plan <pack_plan.json> <out_dir>`.

### Changed

- **Flight file format 2 (`0x0200`):** the header grows from 16 to 24 bytes and is followed by a track directory and the return table, so readers of version 1 must be updated. The packer's `--verify`, its C++ reader and `flight_binary.py` still read version 1 files. On `200_cube`, a file holding the show and its 5 flights home is 174 KB (about 70 KB for the show alone).

### Fixed

- **"Plan it" seemed to do nothing:** planning from a suggestion started at once, but its progress showed at the top of the panel, out of view. It now shows in the row that started it.
- **Weather key fields couldn't be emptied:** clearing a value beside the timeline (wind speed, gust, rain, show time) put 0 back straight away. A field can now be empty while typing, showing the key's value as a placeholder; left empty, the key keeps that value. Typing a show time that moves the key past another one keeps the field focused.
- **The result of a return from inside a move** said the fleet "finished the move it was in"; it now says it flew on to that moment and turned for home there.

### Verified

- 357 automated Python tests pass (348 in 1.2.0), plus the packer's C++ tests for the new layout.
- **`200_cube`, 100 s rain window:** 86 % of the show was covered (short by up to 26 s). The top suggestion, one return from 1:06.6 in the move to Shape_818 estimated at 51 s, closed all of it. Planned in 3.3 min, it passed on its first attempt in 51.31 s, exactly the estimate (closest approach 1.524 m). The show then needs a 97 s window instead of 126 s.
- **Flown through the digital twin** with rain reaching its alert level at 0:45 (4 m/s wind): the fleet flew on to 1:06.6, turned for home and was all home by 1:55.5, 29 s before the rain limit, no drone more than 0.21 m from its slot, lowest battery 86 %.
- **Flight files, `200_cube`:** 200 files with one pack id; every track within 5 mm of its plan (the format's resolution), and every flight home starts within 8 mm of where the show is at its start, so no drone jumps when it switches.

### Known issues

- **The drones can't fly the returns yet:** the flight files carry them, but no firmware reads version 2 and nothing sends the return command to the fleet.
- **A rejected abort point isn't reported in the readiness panel** (only under Stage 2 › Return paths), and the suggestions offer it again.
- **"Plan again at minimum time" hasn't met a real case:** every return planned so far was already at its minimum time.
- **"Pack again" can be asked for needlessly:** the Flight files card compares with the last return-planning job, even a cancelled one that planned nothing new.
- **A return from inside a move is estimated as if the drones started at rest;** one that needs safety-check retries takes longer than its estimate.
- The other known issues of 1.2.0 remain, except that the readiness panel now suggests changes.

## [1.2.0] — 2026-10-04

Shows can now be checked against the weather, and every drone touches a pad only vertically. The desktop app gains a condition simulator: write a weather timeline (wind, gusts, RTK quality, rain), fly the show through the digital twin under it, and see whether the fleet could get home before the rain gets too heavy, using return paths planned from every formation. The stress-test crashes of 1.1.0 were all in the landing; landings, takeoffs and mid-show stops on the pads now go through a hover point straight above the pad, and the 150-, 200- and 500-drone test shows pass the stress test with no crash. Path planning is 3.5–5× faster. In the Blender add-on (1.7.0), the holding area stacks its layers farther apart, and new waiting areas let spare drones wait in the air near the show instead of flying home.

### Added

#### Return paths from every formation (path planning, desktop app)

- **Plan return paths:** after a show passes, Stage 2 can plan one return from each formation to the holding area: the flight home if the show had to stop there. Each starts at the drones' exact positions, speeds and colours in the planned show, lands every drone at rest on a free holding-area slot with LEDs off, and is checked by the same 100-per-second safety check. The first formation's return is the takeoff flown backwards (with the staggered takeoff on, the default). In the desktop app, a "Return paths" section on the Stage 2 tab lists each formation (reached at, flight home, closest approach, status) with "Plan return paths", "Plan" / "Plan again" per row, progress and Cancel. Command line: `studio_bridge stage2-returns <run>`; Python: `drone_core.plan_return_path()`. Results are kept until Stage 2 is run again.
- **Replay a return:** each planned return can be played in the Replay tab as the show up to that formation followed by the flight home, with the abort moment marked.

#### Condition simulator — weather scenarios (desktop app, `twin_sim`)

- **Conditions:** write the weather for a show as a timeline (wind speed, direction and turbulence; gust fronts; RTK quality: fixed, float or GPS only; rain), all free to change during the show. Lanes are aligned with the show's formations; click to add a key, drag to move it, and set exact values beside the timeline. Scenarios are saved with the run; New, Duplicate and Delete.
- **Simulate a scenario:** the whole show is flown once through the digital twin under the timeline, with progress and Cancel, and judged like the stress test (no pair under 0.5 m, every drone lands with at least 15 % battery). The result gives the closest approach, the largest deviation from plan and the lowest battery, each linked to its moment in the playback, and says when the rain reaches its alert and limit levels.
- **Playback with weather:** each drone's planned position next to its simulated one (a red line when they are more than 0.5 m apart), wind streaks, rain, gust fronts sweeping across the field, a heads-up display of wind, rain and RTK, and the largest deviation from plan as a second line on the timeline strip.
- **Digital twin:** time-varying wind, turbulence and RTK quality and any number of gust fronts (`twin_sim/weather.py`), and a scenario runner with recording for playback (`twin_sim/scenario_runner.py`, also a command-line tool). Stress-test results are unchanged.

#### Condition simulator — rain return planning

- **Rain return readiness:** for every moment of the show, the time it would take to get every drone back to the holding area if the rain started then (reaction, finishing the current move, the planned return path, a margin), drawn under the weather timeline against the time the rain takes from its alert to its limit level. It shows the share of the show covered, the uncovered moments and by how much, and the rain window the show needs. Pointing at a moment explains what the fleet would do. Missing return paths can be planned from the panel.
- **The rain rule in the simulator:** when a scenario's rain reaches the alert level, the simulated fleet is called home after the reaction time: it finishes its move and flies the planned return path of that formation. The result says when the return was called, the last drone home, and whether every drone was home before the rain limit (now a pass criterion), and the playback marks the alert, the call and the limit.
- **Fly this:** pick a moment on the readiness chart and simulate rain reaching the alert level then, with the chosen window.
- **Drone profile:** new `environment` keys for the rain alert level, the rain limit level and the reaction time (placeholders until the drone model's water rating is known).

#### Waiting areas (Blender add-on, path planning, desktop app)

- **Waiting areas** (Blender add-on 1.7.0, file format 1.7.0): places in the air, next to the show, where drones a formation doesn't use wait with their LEDs off instead of flying home to the holding area and back. Add any number in the new "Waiting Areas" box (position, size, spacing, safe distance); each is one flat layer, and the last one grows if needed. An area too small for its spare drones grows compactly (both ways, centred), and the panel says to what size; a note appears when a waiting area is a longer trip than going home. Only drones that have already flown use them: every drone takes off from the holding area, and a drone a formation doesn't need yet stays on its pad until its first formation. Path planning sends each spare drone to a nearby area and keeps a waiting drone in its place. Export is locked while an area is too close to a formation or the holding area, overlaps another, or is less than 2 m above the ground. The Studio shows them in the validation panel and in the 3D views; `tools/scripts/convert_waiting_areas.py` adds them to an existing file. Designs without a waiting area work as before.

#### Path planning — reports

- **Timing in the log:** every refining pass in `stage2/log.ndjson` now says where its time went (the drone solves, the pair search, the close-pass checks) and how big it was (pairs, constraints, solve batches).
- **Pad moves per transition:** `metadata.transitions[*].pad_moves` counts the drones that flew to a hover point above their pad, landed, climbed, stayed hovering, or had to reach or leave a pad the old way.

### Fixed

- **Stress-test crashes in the landing:** the return leg (and every return path) now brings each drone to rest 2 m straight above its slot, flying no lower than 1 m above the lowest slot on the way, and then lowers the whole fleet straight down together ("Landing hover height" under Takeoff and landing; 0 = the old landing). Drones used to glide in sideways a few centimetres above the ground, and in the stress test a gust or the downwash of the drones parked above pushed some onto the ground short of their slot, where a neighbour hit them. `500_cube`, `200_cube` and `150_cone` now pass the stress test (0 crash flights, was 97, 3 and 4 of 100); shows are about 5.5 s longer.
- **Takeoffs and mid-show stops on the pads:** the same hover point now applies to every pad visit. At the first takeoff the departing drones climb straight up together before flying off. A drone parking mid-show flies to the hover point above its pad, descends, and climbs straight up again before it leaves; a stop too short for that keeps it hovering above its pad with LEDs off. A parked drone never moves to another pad. Where a straight climb or descent would pass too close to another parked or hovering drone, that drone goes the old way and the transition's report counts it. On `150_cone` and `200_cube` no drone needed the old way and none flew sideways below 1.05 m.
- **Padding on real pads:** drones a formation doesn't use now park on the first places of the whole fleet's holding layout. They used to get a layout computed for that smaller number, which differed from the fleet's when the fleet's holding area had to be widened.

### Changed

- **Holding area: layers 4 m apart, alternate layers shifted** (Blender add-on 1.7.0, file format 1.7.0): parked drones used to stack 2 m straight above each other, so the drones below sat in the downwash of those above and a pad's hover point was the slot of the next layer. New designs stack layers 4 m apart ("Layer Gap") and shift every other layer half a place ("Shift Alternate Layers"), 126 / 100 places per layer at 40 × 10 m. A gap below 3.6 m with more than one layer locks Export. Older files keep their layout. The Studio's capacity gauge and Stage 2 follow the same layout, and `tools/scripts/convert_holding_layout.py` re-lays an existing file.
- **Path planning is 3.5–5× faster:** each refining pass now only considers drone pairs that its largest move could bring within the planning distance, instead of every pair within roughly 5–8 m ("Check only pairs that can meet", on by default; turning it off gives exactly the old plans). Measured on the same files: `500_cube` 3 h 22 min → 44 min, `200_cube` 897 s → 254 s, `150_cone` 1 715 s → 354 s, with the same closest approaches to within 8 mm except one transition of `150_cone` (1.481 m instead of 1.506 m) and `500_cube`'s return leg (1.500 m instead of 1.452 m), all above the 1.45 m check.
- **Desktop app — new look:** every page except Replay and Conditions follows the v3 design (sidebar with the pipeline stages, verdict cards, reworked validation report and Stage 2 page); Replay and Conditions were reworked separately to match. Stage 3 and Conditions are now one Stage 3 tab with sub-tabs.

### Verified

- 348 automated Python tests pass (242 in 1.1.0).
- **Stress test, 100 flights, same seed as before:** `200_cube` 3 → 0 crash flights (worst approach 0.114 → 0.970 m), `150_cone` 4 → 0 (0.239 → 0.813 m), `500_cube` 97 → 0 (0.025 → 0.593 m). Every transition of every show passed its safety check on the first attempt. The largest deviation from plan fell from 5.5 m to 0.91 m (`200_cube`) and from 7.4 m to 1.16 m (`500_cube`).
- **Waiting areas, `150_cone`** (two areas): no drone lands mid-show (was 24), show 332.0 → 297.8 s, 0 crash flights, warning pairs 69 → 40, worst approach 0.653 → 0.850 m, lowest final battery 70.5 → 73.0 %.
- **`300_cube`, the first Blender design with a waiting area** (re-exported with compact growth): show 304.1 → 273.6 s and 94.2 → 82.7 km flown against the first export, worst approach 0.817 m.
- **Return paths, `200_cube`:** all 4 returns passed on their first attempt, 30.7–60.6 s home (up to 146 s with the return leg alone), closest approach 1.502–1.517 m; each starts at the show's position and speed to within 1e-9 m.
- Planning stays deterministic: two runs of the same show and settings give byte-identical plans.

### Known issues

- **Spare drones can still stretch a transition:** with or without waiting areas, a transition is lengthened when spare drones have far to fly, e.g. when drones that waited on their pads since the takeoff leave the holding area for a formation on the other side of the field (`150_cone`: two transitions about twice their design length; `300_cube`: still about twice where its waiting area is ~57 m from the formation).
- **Check Kinematics can flag a false over-speed:** when two keyframes sample a different number of points and the fleet is larger than the shape, the add-on compares a formation point with a holding slot and may lock Export or stretch the timeline after Auto-Fix. Stage 2 is not affected.
- **RTK float near the ground:** in the condition simulator, drones with degraded RTK touched the ground on the old low landing approach and stayed there. This scenario has not been re-run with the hover-point landing; the twin's ground model may be partly to blame.
- **`500_cube` has not been re-planned with the mid-show pad rule** (its first plan with the landing fix passes the stress test).
- **Rain thresholds are placeholders:** the rain alert and limit levels and the reaction time are not those of a specific drone. The readiness panel reports what is not covered but does not yet suggest changes.
- **Jerk is over its limit at the joints between path parts** (12–44 % on the real shows), as in 1.1.0; return paths use the same solver, so this presumably applies to them too.
- **No separate climb and descent speed limits**, and **speed, acceleration and jerk limits are typical values**, not those of a specific drone.
- **Retries are hit and miss**, as in 1.1.0; no retry was needed on the real shows.
- **A safety distance set above about 3 m is not fully checked.** The app warns about such a setting; the default of 1.45 m is not affected.
- **The original 300-drone test file can't be planned:** its first formation overlaps the holding area (the new `300_cube` design plans).
- The 100-drone test shows (`export_100_cone`, `export_100_sphere`) have not been re-checked with this version's planner.
- Windows 10/11 x64 only. The installer made by `npm run tauri build` contains only the app window; computation still needs this repository set up on the same machine.

### Upgrade notes

- Re-run `setup.ps1`: the path planner and the C++ packer must be rebuilt.
- Install the Blender add-on 1.7.0. Files from add-on 1.5.0 and 1.6.0 stay valid and keep their holding layout. To use the new layout or waiting areas on an existing file, re-export it, or use `tools/scripts/convert_holding_layout.py` and `tools/scripts/convert_waiting_areas.py`.
- Files exported before 2026-10-04 whose waiting area had to grow carry the old strip layout: re-export them.
- Plans differ from 1.1.0 (hover-point landings, takeoffs and pad stops; the faster pair search). Set "Landing hover height" to 0 and turn off "Check only pairs that can meet" to come closest to the 1.1.0 plans.
- Return paths belong to a run's Stage 2 result: running Stage 2 again deletes them.

### Requirements

Unchanged from 1.1.0: Visual Studio 2022 or later with "Desktop development with C++", Python 3.14 (64-bit), Node.js 20 or later, Rust, Git. Blender 4.0 or later for show design. An OpenCL driver for the digital twin.

## [1.1.0] — 2026-09-30

Real shows now plan end to end. Both real test shows, 150 and 200 drones, plan with every transition passing the safety check on its first attempt: the path planner was reworked to judge its paths the way the safety check does, to start from flyable paths, and to stop setting up drones to cross where it can't adjust their paths. The Blender add-on gains takeoff and return legs, a ground level and holding-area safety checks, and the digital twin now runs on any GPU (OpenCL instead of NVIDIA Warp).

### Added

#### Show design — Blender add-on (`stage1_designer`, add-on version 1.6.0, export schema 1.6.0)

- **Holding area in the scene:** a **Create in Scene** toggle in the Holding Area box builds the takeoff area as real objects: a wire box for the volume and a sphere per launch slot for the whole fleet, placed where the drones will really be (heading offset included). Grab the box to move the holding area; the panel settings follow. The panel also shows the slot grid and warns when the area had to be widened.
- **Show clearance caution:** a **Safe Distance to Show** setting (default 5 m) in the Holding Area box. Check Kinematics, Auto-Fix and Export check every formation against the holding area and show a caution, per formation, for points inside it or closer than the safe distance. **Export is locked** while any caution stands, like a kinematic error. With **Create in Scene** on, a yellow wire box shows the safe-distance zone.
- **Takeoff and return legs:** a **Takeoff & Return** box sets each leg to **Auto** (as fast as safe) or a **target time**. The panel estimates each leg's minimum time (the longest flight at top speed, with the same margin as Auto-Fix, plus the row-by-row takeoff waves), warns when a target is below it, and sums up takeoff + show + return. The export carries the targets in a new optional `legs` field.
- **Ground level:** a **Ground** box sets the ground height (default z = 0). The panel warns about formation points and parked drones below it, and **Export is locked** until nothing is below the ground. **Show Ground in Scene** draws a wire grid at that height under the show and the holding area. The ground is exported as `ground_z_m`, and the desktop app's replay uses it for its "below ground" list instead of a fixed z = 0.

Files exported by add-on 1.5.0 (schema 1.5.0) stay valid and plan as before.

#### Path planning (`stage2_core_engine`)

- **Takeoff and return legs:** for files with `legs`, Stage 2 flies the takeoff in max(target, minimum time), or the minimum for Auto. It starts the show's timeline when the takeoff reaches the first formation, and adds a **return leg** that lands every drone at rest on a free holding-area slot with its LEDs fading to off, checked by the same 100-per-second safety check. The output reports each leg's actual start, end and duration (`metadata.legs`), and `total_duration_sec` includes the return. Files without `legs` plan exactly as before. The desktop app names the return leg "to the holding area".
- **Holding-area keep-out zone:** Stage 2 routes the show's flight paths around the holding area at the design's **Safe Distance to Show** (`holding_area.show_clearance_m`), and checks it 100 times per second. Drones taking off, landing or parked are exempt. A show it can't route safely, or whose formations are inside the safe distance, is refused with the transition, drone, time and distance. Files without the setting plan as before.
- **Planned vs flown time:** the output lists every transition with its start and end show time, planned and flown duration, and the number of attempts (`metadata.transitions`).

### Fixed

#### Path planning — real shows now plan

- **Near misses:** the planner checked drone spacing only about every 0.6 s and kept every refinement step, good or bad, so it often returned paths it believed were safe but that came too close between its checks, and retries were a matter of luck. It now checks spacing as finely as the final safety check (100 times per second) and keeps a step only if it makes the paths better. The closest pass it reports is now what the safety check measures.
- **Flyable starting paths:** the planner's starting paths broke the speed, acceleration and jerk limits, so its first step on each part of a transition was a large unchecked jump. Each drone's starting path is now moved to the closest path within the limits before planning.
- **Drones crossing at formation points:** drones passed each formation point at speed, each in its own direction, so neighbours coming from different directions could cross just before or after it, where the planner can't adjust the path. All drones now pass a formation point with the same velocity, so near it they move together like the formation itself.
- **Formations passed at a sensible speed:** drones passed a formation at full speed in the direction they arrived from, even when the next formation was somewhere else, so a whole formation had to brake and turn at once. They now pass it with the speed and direction of the formation's own movement from the previous to the next formation: nearly at rest when it stays in place.
- **Drones passing each other where long transitions are split:** a long transition is planned in parts, and at each split every drone was sent straight towards its own target, so two drones passing each other there could come closer than planned just before or after the split, where the planner can't adjust the path. All drones now pass each split with the same velocity.
- **Parked drones stay parked:** when a formation uses only part of the fleet, the drones left in the holding area used to hop up to several meters and land again while the others took off, sometimes close enough to each other or to a departing drone to fail the safety check. They now stay on their pads, and the other drones' paths go around them.
- **Drones parking mid-show:** a drone landing on a holding-area pad during the show used to arrive at flying speed, then slide about 2 m along the ground into the next pad. Every show that parked drones mid-show failed the safety check. Drones now land at rest and stay parked.
- **Staggered takeoff:** with launch rows leaving one after another, the early rows reached the first formation at flying speed and stopped dead to wait for the last row, which no drone can do. The takeoff now arrives at the first formation at rest.

#### Path planning — ground and timing

- **Paths below the ground:** Stage 2 now uses the design's ground level (`ground_z_m`) as an altitude floor. No point of any planned path goes below it, including takeoff, formations passed near the ground and the return. A show whose formations or holding area are below the ground is refused before planning. Files that declare no ground are planned as before.
- **Ground-level takeoffs rejected by a few centimetres:** with the holding area on the ground, planned paths could come out 1–6 cm below it. The safety check then rejected attempts whose drone spacing was fine, and the error only mentioned spacing. Paths now keep a small buffer above the ground and never go below it. When a transition is rejected, the error and the progress report say which check failed on each attempt: drone spacing, ground, or holding-area clearance.
- **Timing after a safety retry:** a transition that passed only on a retry was flown longer than the timeline said. The next transition then started while it was still being flown, LED fades overshot, and the show's reported length (and the desktop app's replay) ended early. Every transition is now timed by the attempt that actually passed.

### Changed

- **Path planning — faster:** each part of a transition stops refining once 6 passes in a row bring no progress, instead of always running all 25. A transition that misses the safety distance badly (below 1 m) is reported at once instead of retried twice with longer times, since more time can't fix it. Both are planner settings in the desktop app ("Stop after passes without progress", "Retry only near misses").
- **Path planning — new planner settings** (all in the desktop app's settings editor and `config/core_config.json`, on by default): "Start from flyable paths" (`repair_seed`), "Shared velocity between transition parts" (`shared_substage_velocity`), "Formation speed from both legs" (`centered_formation_velocity`). "Extra checks per pair" (`cutting_plane.max_dynamic_collocations_per_pair`) now defaults to 0, meaning no cap (it was 3).
- **Path planning — progress and rejection details:** every planning pass reports whether it was kept, its step size limit and the best closest pass so far; every safety-check attempt reports which checks it passed (spacing, ground, holding-area clearance) and its lowest point. A rejected show's report lists these per attempt too.
- The holding area now counts as its whole declared volume plus the parked grid (padded by half a grid step), in both the add-on and the desktop app's "formation overlaps the holding area" warning. Before, the app only counted the parked grid, which a small fleet fills only partly.
- **Digital twin — runs on any GPU:** the stage 3 simulation now uses OpenCL instead of NVIDIA Warp, so it runs on Intel, AMD and NVIDIA graphics (including integrated graphics) or on every core of the CPU. On an Intel Iris Xe laptop, 1 000 drones simulate about ten times faster than real time (before: half real time), and a 20-flight stress test of a 100-drone show takes 14 s instead of almost 6 minutes. Results match the Warp version (checked against a recorded Warp run); weather flights differ only in their random draws. **Needs an OpenCL driver:** any graphics driver has one; without a GPU, install Intel's CPU Runtime for OpenCL Applications (see `README.md`). `warp-lang` is replaced by `pyopencl` in `requirements.txt`.
- **Digital twin — command line:** the module is now `stage3_simulation_packer.twin_sim` (was `warp_sim`). `--device` takes `auto`, `gpu`, `cpu` or an `opencl:P:D` id (list them with `tools\scripts\benchmark_stage3.py --list`). `--batch` (flights simulated together, automatic by default) replaces `--workers`.
- **Desktop app — stress test:** "Simulate on" lists devices by name (for example "Automatic (Intel Iris Xe Graphics)"); the "flights at once" setting is gone because flights are batched automatically. Without any simulation device, the app says what to install.

### Verified

- 242 automated Python tests pass (158 in 1.0.0), the Stage 2 C++ unit tests pass, and the Stage 2 smoke test passes (closest pass 1.968 m).
- **Real 150-drone show** (`export_150_cone`: 5 formations, a ground-level holding area, drones parking and relaunching mid-show): plans end to end, every one of its 6 transitions on the first attempt, closest pass 1.496 m, show 322.3 s, 42.4 minutes of planning. Before this release's planner fixes it failed at its third transition.
- **Real 200-drone show** (`export_200_cube`: 5 formations): plans end to end, every one of its 6 transitions on the first attempt, closest pass 1.496 m, show 177.5 s, 21.8 minutes of planning. Before this release's planner fixes it failed at its second transition.
- Planning is deterministic: the same show and settings give the same paths, to the millimetre.

### Known issues

- **Jerk is over its limit at the joints between path parts:** at formation points, where long transitions are split, and at the ends of the takeoff and return, planned paths change their acceleration faster than the 5 m/s³ jerk limit, by 12–44 % on the real shows (up to 7.2 m/s³). Speed (at most 4.1 m/s) and acceleration (at most 2.8 m/s²) stay within their limits. Jerk is mostly a smoothness setting; the likely effect is slightly larger tracking error at those moments, which the digital twin can show. Nothing checks it before flight.
- **No separate climb and descent speed limits:** one speed limit applies in every direction, so descents up to about 2.5 m/s are planned without warning. Show practice usually keeps descents slower.
- **Speed, acceleration and jerk limits are typical values, not those of a specific drone.** Set your drones' real limits in the design file or in `config/core_config.json` before flying.
- **Retries are hit and miss:** a transition that fails its safety check is re-planned with more time, which can come out better or worse. On both real shows no retry was needed.
- **A safety distance set above about 3 m is not fully checked.** The app warns about such a setting; the default of 1.45 m is not affected.
- **The real 300-drone test file can't be planned:** its first formation overlaps the holding area. The add-on now refuses to export such a design; the file has to be redesigned.
- The 100-drone test shows (`export_100_cone`, `export_100_sphere`) have not been re-checked with this version's planner.
- The replay samples 20 frames per second, so its closest distance can differ from the 100 Hz safety check by a few millimetres.
- Windows 10/11 x64 only. The installer made by `npm run tauri build` contains only the app window; computation still needs this repository set up on the same machine.

### Upgrade notes

- Re-run `setup.ps1`: `pyopencl` replaces `warp-lang`, and the path planner must be rebuilt.
- Scripts calling `stage3_simulation_packer.warp_sim` must use `stage3_simulation_packer.twin_sim`, and `--batch` instead of `--workers`.
- Show files from add-on 1.5.0 plan as before. Re-export with add-on 1.6.0 to use takeoff and return legs, the ground level and the holding-area safe distance.
- The same show plans to different (safer) paths than in 1.0.0; results from 1.0.0 are not reproduced.

### Requirements

Visual Studio 2022 or later with "Desktop development with C++", Python 3.14 (64-bit), Node.js 20 or later, Rust, Git. Blender 4.0 or later for show design. An OpenCL driver for the digital twin: any current graphics driver (Intel, AMD, NVIDIA), or Intel's CPU Runtime for OpenCL Applications on a machine without a GPU.

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

[1.1.0]: https://github.com/Brakenull/drone-show-studio/releases/tag/v1.1.0
[1.0.0]: https://github.com/Brakenull/drone-show-studio/releases/tag/v1.0.0
