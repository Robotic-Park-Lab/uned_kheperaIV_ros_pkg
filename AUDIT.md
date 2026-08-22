# Audit — uned_kheperaIV_ros_pkg (2026-08-22)

Checklist grounded in real inspection of `humble-dev` (not guessed from memory): `colcon build`/`colcon test` of the 4 `ament_python` packages, `git log`/`git ls-tree` across `humble-dev` and `benchmark`, and reading the actual source of every package touched below. Following the same discipline used on `uned_crazyflie_ros_pkg`'s audit: this one folds in, from the start, both the "clean up what's broken" pass (equivalent to that repo's Fase 1) **and** the "make it English-documented, tested, and modular" pass (equivalent to its Fase 2), since that's what you asked for this time.

`benchmark` is **locked and out of scope for any code change** — it backs a published book chapter (same hard rule as `uned_crazyflie_ros_pkg`). Everything below only touches `humble-dev` (and `doc` for this document itself). Where `benchmark` is relevant, it's only read from, never written to.

Mark items `[x]` as they're resolved. Work started 2026-08-22 on branch `refactor/modular-restructuring`, after Francisco reviewed this document and gave the go-ahead for everything below (with the clarifications recorded inline), same review-before-merge pattern as the Crazyflie repo — this branch is not yet merged to `humble-dev`.

## 0 — Blocking decision: `libkhepera-2.1` licensing — RESOLVED

You asked to add `libkhepera-2.1` (used to build the C program that runs on the Khepera's onboard Linux, `scripts/server.c`) to the repo "si es lícito". Checked the actual library folder you left in this session's working directory: **no `LICENSE`/`COPYING` file anywhere in it**, and every source file header reads `Copyright (C) 2013 K-TEAM SA` with no redistribution grant of any kind. This repo is **public** and BSD-3-Clause licensed. **Decision (confirmed by Francisco)**: don't vendor the source. Resolved in `981975c`: `scripts/Makefile` builds `server.c`/`prog-template.c` against `libkhepera` expecting it at a documented `LIBKHEPERA_PATH` env var, without the vendor source itself living in this repo — verified for real, both `.c` files compile clean against Francisco's actual `libkhepera-2.1` headers (final ARM linking not tested, that build is ARM-only and this sandbox is x86_64).

## 0b — Urgent, found while working: leaked WiFi credential — RESOLVED

Not part of the original audit — found while reviewing `scripts/` for the Makefile above. A real WiFi password (`MOVISTAR_D647`) was hardcoded in plaintext in 3 files (`scripts/arduino-khepera-uROS/src/main.cpp`, `scripts/Arduino/test_ros2/test_ros2.ino`, `scripts/Arduino/test_wifi/test_wifi.ino`), committed several commits deep, in this public repo. Flagged to Francisco immediately rather than waiting. **Francisco confirmed the network is old/retired** — no git-history purge requested, just a clean `HEAD` going forward. Resolved in `981975c`: replaced with `SECRET_SSID`/`SECRET_PASS` from a gitignored `arduino_secrets.h` (same pattern already used in `test_wifi/ConnectNoEncryption/`), `.h.example` templates committed instead, `arduino_secrets.h` added to `.gitignore`.

## 1 — Metadata cleanup (all 8 packages) — RESOLVED (`e68dcb4`)

- [x] **`TODO: License declaration`** filled in all 8 `package.xml` → `BSD-3-Clause`.
- [x] **`TODO: Package description`** filled in all 8 with a real, specific description.
- [x] **Maintainer inconsistency** unified to `Francisco José Mañas Álvarez <fma527@ual.es>` across all 8 `package.xml`.
- [x] **Stray 0-byte file** `uned_kheperaiv_gui/uned_kheperaiv_gui/sys` removed.
- [x] **`uned_kheperaiv_webots` marker warning** fixed: split `resource/` (now only the ament marker) from a new `resources/` (the real data files: `default_config.yaml`, urdf files, `ros2control.yml`) — verified gone with a real `colcon build`. Also found while doing this: those `resources/` files look unreferenced anywhere in the repo (real robot description flows through `uned_kheperaiv_config/urdf/khepera.xml` instead) — not deleted without your confirmation, flagged here as **pending your review**.

## 2 — Real bugs and dependency gaps — RESOLVED (`e68dcb4`)

- [x] **Real bug in `experience.launch.py`**: `for robot_id in physical_khepera_list:` iterated a comma-joined string character by character. Fixed: `physical_khepera_list` is now a real Python list, verified with a synthetic 2-physical-robot experience producing exactly 2 driver nodes (was one broken node per character before).
- [x] **Two more real bugs found while verifying the fix above by actually running `get_ros2_nodes()` against all 3 existing experience `.yaml`, not just reading the code**: `Architecture.node['file']` and `Robots[x]['camera']` were read unconditionally, crashing with `KeyError` on `Demo_formation_central_webots.yaml` (missing both keys). Both now use `.get()` with a sensible fallback (the experience file itself; `'false'`) — verified, all 3 existing experiences now generate their launch actions without error.
- [x] **Undeclared dependencies**: real `<depend>`/`<exec_depend>` added to all packages missing them, including `multi_agent_pkg` in `uned_kheperaiv_webots`.
- [ ] **Lint tests failing** (10 of 12): not yet fixed directly — folded into the per-package work in section 4 below (each package's own pass excludes generated code from lint and fixes real style issues as part of adding tests/README).

## 3 — Duplicated / dead / unclear code

- [ ] **`uned_kheperaiv_gui`** dual-implementation pattern: **in progress**, delegated to a dedicated pass, same fix as `uned_crazyflie_gui`.
- [ ] **`uned_kheperaiv_task`** duplication (`distance_based_formation_control.py`/`shape_based_formation_control.py`, plus `gazebo_driver.py` which turned out to have its own `Agent`/`KheperaIVDriver`/`PIDController` too — **3-way duplication, not 2-way as first estimated**): **in progress**, delegated to a dedicated pass. Note: `gazebo_driver.py` stays as a real, maintained node (see the Gazebo decision below) — the dedup must preserve its behavior exactly, not treat it as legacy.
- [x] **`uned_kheperaiv_controllers` missing from `humble-dev`**: **RESOLVED** in `b26d050` — ported from `benchmark` (read-only, `benchmark` untouched), metadata fixed, verified with a real `colcon build`.
- [x] **`turtlebot3burger_driver.py`**: Francisco confirmed — **removed**, resolved in `e68dcb4`.
- [x] **Gazebo vs. Webots split — decision made, not "retire"**: Francisco wants Gazebo **updated to work with the current Gazebo** (Gazebo Classic 11, the version that pairs with ROS 2 Humble), not retired. Also decided: `uned_khepera_description` didn't make sense standalone → **merged into `uned_kheperaiv_config`** (resolved in `75acf87`, verified with a real `colcon build` — 40 generated `kheperaXX.urdf` + all mesh files install correctly). `uned_khepera_gazebo` → **renamed to `uned_kheperaiv_gazebo`** for naming consistency (also `75acf87`). The actual Gazebo Classic 11 compatibility work (verifying/fixing the `diff_drive`/`p3d` plugins and the `spawn_entity`/model-database flow with a real `gzserver`) is **in progress, delegated to a dedicated pass — marked in that pass's own commit/README as done entirely by Claude, not yet validated by Francisco on his own setup.** `uned_vicon_gazebo` (Vicon-to-Gazebo bridge) left as-is for now, revisit once the Gazebo path itself is confirmed working.

## 4 — The broader pass (English docs, tests, driver consolidation)

- [x] Root work started: metadata, launch bugs, credential leak, `libkhepera` Makefile, gazebo rename+description merge, controllers port — all above, all on `refactor/modular-restructuring`.
- [ ] **English READMEs + tests per package**: `uned_kheperaiv_driver`+`uned_kheperaiv_webots` (also breaking a real found dependency: `uned_kheperaiv_webots/khepera_driver.py` imported `Agent`/`Crazyflie_ROS2` from **the Crazyflie repo** — `Agent` genuinely used for formation tracking, `Crazyflie_ROS2` a dead import. Francisco confirmed: give Khepera its own copy, breaking the cross-repo dependency — in progress), `uned_kheperaiv_gui` (in progress, same pass as the dedup above), `uned_kheperaiv_task` (in progress), `uned_kheperaiv_gazebo` (in progress, same pass as the Gazebo update). Root README, `scripts/README.md`, `uned_kheperaiv_config/README.md`, `uned_kheperaiv_controllers/README.md` handled directly, not delegated.
- [x] **`experience.launch.py` bugs**: fixed above (section 2), all 3 existing experience `.yaml` verified against it.
- [ ] **`scripts/` review**: `Makefile` done (section 0). Arduino/ESP32 micro-ROS prototypes and the 2 Matlab files still need your manual "keep vs. retire" call, same as the Crazyflie repo's `scripts/` — not touched beyond the credential fix.

## How this maps to the Crazyflie repo's phases, for reference

Section 1 ≈ Crazyflie's Fase 1 "Simples". Section 2 ≈ Fase 1 "Medias". Section 3 ≈ Fase 1 "Complejas" + the missing-controllers-package finding (no Crazyflie equivalent — that repo never lost a package between branches). Section 4 ≈ Crazyflie's Fase 2, done as one pass instead of two this time, per your request.
