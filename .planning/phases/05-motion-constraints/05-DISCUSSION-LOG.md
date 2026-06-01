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

---

## Area 10: ROS2 float64 infinity for workspace defaults

**Question:** ROS2 float64 parameters don't support Python float('inf'). How should workspace bound defaults signal 'unconstrained'?

| Option | Selected |
|--------|----------|
| Large float sentinels (-1e9 / +1e9) | ✅ |
| Add constraints.workspace.enabled boolean | — |
| 0.0 defaults with all-zeros-means-disabled | — |

**Notes:** ConstraintConfigDTO treats workspace as disabled when sentinel range ≥ 2e9 on any axis.

---

## Area 11: Per-joint speed constraints

**User freetext:** "joints should also have speed constraints array (lower/upper)"

**Question 1:** Shape of joint speed constraints?

| Option | Selected |
|--------|----------|
| Per-joint velocity limits (rad/s) via max_velocities[] | ✅ |
| Single node-level max joint speed scalar | — |
| Per-joint array + global fallback | — |

**Question 2:** Share names[] array or separate?

| Option | Selected |
|--------|----------|
| constraints.joint.max_velocities[] matching names[] | ✅ |
| Separate constraints.joint_velocities.names + .values | — |

**Question 3:** Enforcement?

| Option | Selected |
|--------|----------|
| Planning-time only — passed to MoveIt2, no goal rejection | ✅ |
| Goal validation + MoveIt2 joint velocity constraints | — |

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

- **[RESEARCH-02]** Correct MoveIt2/PILZ API for per-joint velocity overrides in
  `MotionSequenceItem` (for `constraints.joint.max_velocities[]`).
  Candidates: `MotionPlanRequest` velocity scaling, `JointConstraint` velocity field,
  or robot model override mechanism.

---
*Generated: 2026-06-01 (updated with Areas 10–11)*
