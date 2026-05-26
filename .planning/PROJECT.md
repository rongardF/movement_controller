# movement_controller

## What This Is

A ROS2 Jazzy package that provides a clean, vendor-agnostic API for controlling industrial robot arms using MoveIt2. It exposes a single ROS2 action for executing multi-path blended trajectories and a service interface for managing the robot's collision scene. The initial implementation targets the Universal Robotics UR10 arm via `ur_robot_driver`, with the architecture designed for future vendor extensibility.

## Core Value

A single reliable ROS2 action that executes collision-aware, blended multi-path trajectories on a UR10 using the PILZ motion planner — working identically in Gazebo simulation and on real hardware.

## Requirements

### Validated

(None yet — ship to validate)

### Active

- [ ] ROS2 package scaffold with correct `ament_cmake_python` structure, CMakeLists.txt, package.xml, and Python module layout
- [ ] ROS2 action interface (`ExecuteTrajectory`) with trajectory path list goal, per-path feedback (status + UUID4 path ID), and success/error result
- [ ] Execute multi-path trajectory using PILZ planner (LIN / PTP / CIRC motion types) via `MoveGroupSequence` with blend radius support
- [ ] Look-ahead planning: plan the next trajectory path in the background while the current one executes, minimising inter-path latency
- [ ] Scene management interface — add/remove collision objects (primitives and mesh files), both free-standing and attached/detached to robot links
- [ ] Persistent motion constraints configured via node parameters: per-joint constraints, end-effector orientation constraints, workspace bounding-box constraint
- [ ] UR10 integration via `ur_robot_driver` and `ur_moveit_config` packages; launch files for both simulation and real hardware modes
- [ ] End-to-end validated in Gazebo Harmonic simulation with UR10 URDF/SRDF
- [ ] End-to-end validated on real UR10 hardware

### Out of Scope

- Multi-vendor support beyond UR10 in v1 — abstract base designed now, other vendors deferred
- UR10e (e-Series) — targeting UR10 classic; e-Series support deferred
- Per-move constraints in the action goal — constraints are persistent node parameters only
- Force/torque control — pure motion planning, no compliant control
- Gripper / end-effector actuation — out of scope for movement controller
- REST / HTTP API layer — consumers are other ROS2 nodes only

## Context

- **Runtime:** ROS2 Jazzy on Ubuntu 24.04, developed inside a Docker devcontainer (`ros:jazzy-ros-base`)
- **Motion planning:** MoveIt2 with PILZ Industrial Motion Planner plugin for deterministic LIN/PTP/CIRC motion profiles; `MoveGroupSequence` action for blended execution
- **Look-ahead planning:** While path N executes, path N+1 is planned concurrently; planned trajectory is queued and executed immediately on path completion to minimise stop-start latency
- **Driver:** `ur_robot_driver` (Universal Robotics official ROS2 driver); simulation via `fake_hardware_interface`
- **Data models:** Pydantic v2 for all internal DTOs; ROS2 `.action` / `.msg` / `.srv` files for interface definitions
- **Language:** Python 3.11+ primary; C/C++ only for `rosidl` interface generation
- **Testing:** `pytest` + `ament_pytest`; hardware interfaces mocked in unit tests

## Constraints

- **Tech stack:** MoveIt2 Python bindings (`moveit_py`) — not MoveIt Commander (ROS 1 only)
- **Driver:** Must use vendor-provided `ur_robot_driver`; no custom UR protocol implementation
- **Interface format:** ROS2 action/service/message files only — no Python class substitutes for ROS2 interfaces
- **Constraints scope:** Motion constraints are persistent node parameters; not overridable per action call
- **Licensing:** BSD-3-Clause; all source files must carry the license header
- **Simulation:** Gazebo Harmonic (paired with ROS2 Jazzy); `use_sim_time` parameter must be respected

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| PILZ planner over OMPL | Deterministic, predictable motion profiles (LIN/PTP/CIRC); OMPL is stochastic and unsuitable for industrial paths | — Pending |
| `MoveGroupSequence` for blending | Native MoveIt2 mechanism for blended multi-segment trajectories; avoids stop-start between paths | — Pending |
| Look-ahead parallel planning | Minimises inter-path latency without changing the action interface; background thread plans ahead during execution | — Pending |
| Constraints as node parameters | Constraints reflect robot cell configuration (physical workspace, safety limits); they don't change per-move | — Pending |
| UR10 only in v1, abstract base now | Avoids premature abstraction cost while protecting architecture; `BaseMovementController` designed but only UR10 implemented | — Pending |
| Vendor driver dependency | Reuse `ur_robot_driver` and `ur_moveit_config` rather than reimplementing; follow their `moveit_ros_move_group` pattern | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-05-26 after initialization*
