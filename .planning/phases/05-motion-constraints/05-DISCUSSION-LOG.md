# Phase 5: Motion Constraints — Discussion Log

**Phase:** 5 — Motion Constraints
**Date:** 2026-06-01
**Areas discussed:** 5 initial + 4 additional = 9 total

---

## Area 1: Constraint Ownership & Injection

**Question:** Who owns the built Constraints objects and attaches them to planning requests?

| Option | Selected |
|--------|----------|
| Controller configures, planner injects | ✅ |
| Controller builds & passes Constraints objects | — |
| New ConstraintService class | — |

**Follow-up:** Where in the MotionSequenceRequest do constraints get attached?

| Option | Selected |
|--------|----------|
| MotionSequenceItem.constraints (path constraints) | ✅ |
| set_path_constraints() on planning component | — |
| Both — belt and suspenders | — |

**Agent discretion:** URMovementController passes ConstraintConfigDTO via `set_constraints()` method; PilzPlannerService injects per-item in `_build_sequence_request()`.

---

## Area 2: Parameter Namespace Structure

**Question:** Dot-namespaced vs flat underscore?

| Option | Selected |
|--------|----------|
| Dot-namespaced (constraints.workspace.x_min) | ✅ |
| Flat with underscores | — |

**Notes:** ROS2 parameter dot notation keeps parameters grouped in tooling.

---

## Area 3: Speed & Acceleration Enforcement

**Question 1:** How should CON-05/CON-06 be enforced?

| Option | Selected |
|--------|----------|
| Goal rejection in _goal_callback | ✅ |
| Velocity scaling at planning time | — |
| Both | — |

**Question 2:** What does cartesian_speed=0.0 mean?

| Option | Selected |
|--------|----------|
| 0.0 means unconstrained — skip check | initial selection |

**User freetext:** "I think right now we are not controlling the cartesian speed of the movement. I want it so that the 'cartesian_speed' value that is provided in each 'TrajectoryPath.msg' is used for performing the cartesian movement with the selected link."

**Clarified decision:**
- Per-path `cartesian_speed` is passed to PILZ for actual execution speed control
- Goal is **rejected** (not capped) if it exceeds the node-level max
- User asked about `LimitMaxCartesianLinkSpeed` planning adapter — research needed

**Question 3:** Does the same pattern apply to acceleration?

| Option | Selected |
|--------|----------|
| Same pattern as cartesian_speed | ✅ |
| Acceleration out of scope | — |

---

## Area 4: Joint Constraint Parameterization

**Question 1:** How structured?

| Option | Selected |
|--------|----------|
| Named per-joint with .lower/.upper | — |
| Array of 6 values in joint order | — |
| Two arrays (lower_limits[], upper_limits[]) | ✅ |

**Question 2:** How are joint names mapped?

| Option | Selected |
|--------|----------|
| Add constraints.joint.names string[] parameter | ✅ |
| Implicit URDF order | — |
| Both empty by default, names required when set | — |

---

## Area 5: Constraint Disable Behavior

**Question:** How can constraint types be individually disabled?

| Option | Selected |
|--------|----------|
| Sentinel default values signal disabled | ✅ |
| Explicit enabled booleans per constraint type | — |
| Off by default, explicit opt-in | — |

---

## Area 6: Invalid Constraint Parameter Handling

**Question:** What happens if constraint parameters are invalid?

| Option | Selected |
|--------|----------|
| Fail on_configure with clear error (ValidationError) | ✅ |
| Skip invalid constraint, warn and continue | — |

---

## Area 7: Orientation Constraint Reference Link

**Question:** Which link does the orientation constraint apply to?

| Option | Selected |
|--------|----------|
| Always tool0 (hardcoded) | — |
| Configurable via parameter | — |

**User freetext:** "It always applies to the link that is used when performing cartesian movement (specified in TrajectoryPath with 'tool_frame' field)."

**Captured decision:** `OrientationConstraint.link_name` = path's `tool_frame` (dynamic, per `MotionSequenceItem`).

---

## Area 8: Lifecycle Re-configure Behavior

**Question:** When on_configure runs after lifecycle restart, how are constraints handled?

| Option | Selected |
|--------|----------|
| Rebuild fresh on each on_configure | ✅ |
| Cache and rebuild only if params changed | — |

---

## Area 9: Speed Violation Error Message Format

**Question:** Error message format for speed violations?

| Option | Selected |
|--------|----------|
| Include path_id, actual value, limit, and param name | ✅ |
| Generic rejection message | — |

**Format locked:** `"Path '{path_id}' cartesian_speed {actual} m/s exceeds node maximum {max} m/s (constraints.max_cartesian_speed)"`

---

## Deferred Ideas

_(None captured during discussion)_

---

## Open Research Items

- **[RESEARCH-01]** Correct PILZ/`GetMotionSequence` mechanism for per-path cartesian speed:
  - `LimitMaxCartesianLinkSpeed` planning request adapter
  - `MotionPlanRequest` velocity fields (e.g. `max_velocity_scaling_factor`)
  - `CartesianSpeedLimitedConstraint` in `Constraints` message
  Researcher must determine which mechanism correctly controls the cartesian speed of
  a specified link in a PILZ `MotionSequenceItem`.

---
*Generated: 2026-06-01*
