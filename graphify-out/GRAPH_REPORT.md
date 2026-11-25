# Graph Report - Hexapode_ECE  (2026-11-25)

## Corpus Check
- Corpus is ~27,726 words - fits in a single context window. You may not need a graph.

## Summary
- 318 nodes · 707 edges · 10 communities (9 shown, 1 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 50 edges (avg confidence: 0.85)
- Token cost: 215,157 input · 0 output

## Community Hubs (Navigation)
- leg_gui Interface (LegApp)
- Torque Tests & Load Analysis
- leg_server C++ Servo Bus
- Dynamixel Docs & TCP Protocol
- Hexapod Kinematics Model
- test_torque Lift Sequence
- Servo Panel & Joint Conversion
- Pi Setup & Project Overview
- LegClient TCP Client
- Power PCB Drill Maps

## God Nodes (most connected - your core abstractions)
1. `LegApp` - 39 edges
2. `ServoPanel` - 21 edges
3. `TorqueTest` - 18 edges
4. `handle()` - 17 edges
5. `hexapod_hardware ros2_control plugin (C++)` - 15 edges
6. `save_config()` - 10 edges
7. `LegClient` - 10 edges
8. `load_config()` - 9 edges
9. `leg_fk()` - 8 edges
10. `plot_test()` - 8 edges

## Surprising Connections (you probably didn't know these)
- `Chart: supply voltage and servo temperature` --conceptually_related_to--> `Alim_SDK_hex_pcb power-supply board`  [AMBIGUOUS]
  Dyna_test/graphs/tension_temperature.png → PCB/Alim_SDK_hex_pcb/jlcpcb/gerber/Alim_SDK_hex_pcb-PTH-drl_map.pdf
- `hexapod_hardware ros2_control plugin (C++)` --references--> `raw_to_joint()`  [EXTRACTED]
  tps_archi.md → Dyna_test/gui/hexapod_model.py
- `hexapod_hardware ros2_control plugin (C++)` --references--> `joint_to_raw()`  [EXTRACTED]
  tps_archi.md → Dyna_test/gui/hexapod_model.py
- `DynamixelSDK C++ installed in /usr/local` --semantically_similar_to--> `DynamixelSDK (ROBOTIS C++)`  [INFERRED] [semantically similar]
  setup.md → Dyna_test/dyna.md
- `udev rule 99-u2d2.rules` --semantically_similar_to--> `udev rule 99-u2d2.rules`  [INFERRED] [semantically similar]
  setup.md → Dyna_test/dyna.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Current servo control stack (GUI/test -> TCP -> leg_server -> U2D2 -> AX-12A)** — dyna_test_gui_leg_gui, dyna_test_gui_test_torque, dyna_test_dyna_leg_server_tcp_protocol, dyna_test_src_leg_server, dyna_test_dyna_u2d2, readme_dynamixel_ax_12a [EXTRACTED 1.00]
- **Target ROS 2 control stack** — tps_archi_hexapod_control, tps_archi_ros2_control, tps_archi_hexapod_hardware, tps_archi_hexapod_description, tps_archi_hexapod_bringup, tps_archi_hexapod_web [EXTRACTED 1.00]
- **Torque calculation and lift validation** — dyna_test_torque_calcul_lever_arm_torque_model, dyna_test_torque_calcul_femur_torque_by_configuration, dyna_test_torque_calcul_trial1_compliance, dyna_test_torque_calcul_trial3_six_leg_lift, dyna_test_gui_test_torque, dyna_test_gui_plot_torque, dyna_test_torque_calcul_measurement_logs [EXTRACTED 1.00]
- **Charts from torque trial 20261125_111436 (feet at 170 mm)** — dyna_test_graphs_charge_femurs_temps, dyna_test_graphs_charge_par_patte_debout, dyna_test_graphs_lever_charge_vs_descente [EXTRACTED 1.00]
- **Femur torque sizing: model, limits, measurement** — dyna_test_graphs_couple_theorique_femur_torque_formula, dyna_test_graphs_couple_theorique_ax12a_torque_limits, dyna_test_graphs_charge_par_patte_debout_femur_dominant_load, dyna_test_graphs_couple_theorique_tripod_feasibility [INFERRED 0.75]
- **Compliance tuning: tested, chosen, applied in GUI** — dyna_test_graphs_reglages_compliance, dyna_test_graphs_reglages_compliance_retained_setting, dyna_test_img_interface_leg_gui_compliance_controls [INFERRED 0.85]

## Communities (10 total, 1 thin omitted)

### Community 0 - "leg_gui Interface (LegApp)"
Cohesion: 0.09
Nodes (4): save_config(), LegApp, side(), resync()

### Community 1 - "Torque Tests & Load Analysis"
Cohesion: 0.07
Nodes (33): Joint offset (mounting offset), Femur load per leg over time (chart), Torque test phases (rapprocher/lever/tenir/redescendre), Standing load per leg and joint (bar chart), Leg to servo ID mapping (coxa/femur/tibia), Theoretical femur torque vs lever arm (chart), AX-12A torque limits (1.5 N.m @12V, 1.33 N.m @10.6V, ~30% continuous), Femur torque model T = (m g / n) x d_f (+25 more)

### Community 2 - "leg_server C++ Servo Bus"
Cohesion: 0.10
Nodes (23): apply_settings_file(), clampi(), comm_err(), current_target(), ensure_port(), handle(), is_known(), main() (+15 more)

### Community 3 - "Dynamixel Docs & TCP Protocol"
Cohesion: 0.08
Nodes (31): AX-12A control table registers, DynamixelSDK (ROBOTIS C++), Foot-tip inverse kinematics, leg_gui_config.json, leg_server TCP protocol (port 5555), Legacy programs mv_hard_code.cpp / my_read_write.cpp, Lift settings trial (slope 8 / punch 100), Dyna_test Makefile (+23 more)

### Community 4 - "Hexapod Kinematics Model"
Cohesion: 0.09
Nodes (17): Bent tibia (curvature + tip offset), Leg layout and servo IDs (6 legs x coxa/femur/tibia), all_ids(), body_to_leg(), _copy(), deg_to_raw(), joint_to_raw(), leg_fk() (+9 more)

### Community 5 - "test_torque Lift Sequence"
Cohesion: 0.12
Nodes (5): body_ik(), Abort, main(), Robot, TorqueTest

### Community 6 - "Servo Panel & Joint Conversion"
Cohesion: 0.13
Nodes (4): raw_to_deg(), raw_to_joint(), safety_reason(), ServoPanel

### Community 7 - "Pi Setup & Project Overview"
Cohesion: 0.15
Nodes (15): Dynamixel AX-12A servos, Hexapode ECE robot, PCB power / Dynamixel board (KiCad), Raspberry Pi 4 (Ubuntu 22.04 / ROS 2 Humble), Git sync PC <-> Pi via GitHub (deploy key), Network dropout diagnosis, Pi NetworkManager config (static 192.168.137.10 + DHCP, IPv6 link-local), No RTC clock on Pi 4 (+7 more)

### Community 9 - "Power PCB Drill Maps"
Cohesion: 0.67
Nodes (3): Alim_SDK_hex_pcb power-supply board, Alim_SDK_hex_pcb NPTH drill map, Alim_SDK_hex_pcb PTH drill map

## Ambiguous Edges - Review These
- `Body lift: load at each step vs foot height (chart)` → `High single-servo load before ground contact`  [AMBIGUOUS]
  Dyna_test/graphs/lever_charge_vs_descente.png · relation: rationale_for
- `Chart: supply voltage and servo temperature` → `Alim_SDK_hex_pcb power-supply board`  [AMBIGUOUS]
  Dyna_test/graphs/tension_temperature.png · relation: conceptually_related_to

## Knowledge Gaps
- **16 isolated node(s):** `Dynamixel AX-12A servos`, `Raspberry Pi 4 (Ubuntu 22.04 / ROS 2 Humble)`, `PCB power / Dynamixel board (KiCad)`, `Password reset via SD card cmdline.txt script`, `ROS 2 Humble on the Pi` (+11 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 84 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LegApp` connect `leg_gui Interface (LegApp)` to `Hexapod Kinematics Model`?**
  _High betweenness centrality (0.199) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `hexapod_hardware ros2_control plugin (C++)` (e.g. with `U2D2 automatic reconnection` and `leg_server.cpp`) actually correct?**
  _`hexapod_hardware ros2_control plugin (C++)` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Dynamixel AX-12A servos`, `Raspberry Pi 4 (Ubuntu 22.04 / ROS 2 Humble)`, `PCB power / Dynamixel board (KiCad)` to the rest of the system?**
  _16 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `leg_gui Interface (LegApp)` be split into smaller, more focused modules?**
  _Cohesion score 0.08636363636363636 - nodes in this community are weakly interconnected._
- **Why does `ServoPanel` connect `Servo Panel & Joint Conversion` to `leg_gui Interface (LegApp)`, `Hexapod Kinematics Model`?**
  _High betweenness centrality (0.112) - this node is a cross-community bridge._

### Low-confidence Hints
_AMBIGUOUS edges — the extractor was unsure. Verify before acting on these._

- **What is the exact relationship between `Body lift: load at each step vs foot height (chart)` and `High single-servo load before ground contact`?**
  _Edge tagged AMBIGUOUS (relation: rationale_for) - confidence is low._
- **What is the exact relationship between `Chart: supply voltage and servo temperature` and `Alim_SDK_hex_pcb power-supply board`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._