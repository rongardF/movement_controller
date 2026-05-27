# Phase 2: LifecycleNode & Action Server Skeleton — Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-27
**Phase:** 2 — LifecycleNode & Action Server Skeleton
**Areas discussed:** Parameter scope, Goal validation boundary, Stub execution behavior, Concurrent goal rejection

---

## Parameter Scope

| Option | Description | Selected |
|--------|-------------|----------|
| Declare Phase 2 params only | on_configure only declares parameters this phase actually uses | ✓ |
| Declare all future params as stubs | Declare every planned parameter now with defaults (workspace bounds, robot_ip, etc.) | |
| Declare Phase 2 + universal/stable params only | Phase 2 params + clearly universal ones needed from startup | |

**User's choice:** Declare Phase 2 params only

**Follow-up — Which params?**

| Option | Description | Selected |
|--------|-------------|----------|
| Minimal: action_server_name only | Only the action server name | |
| action_server_name + moveit_group_name | Both strings, both defaulted — moveit_group_name is stable and needed from Phase 3 | ✓ |
| Just action_server_name, explicit comment | action_server_name only with code comment for future | |

**User's choice:** `action_server_name` + `moveit_group_name`

**Follow-up — Default for action_server_name?**

| Option | Description | Selected |
|--------|-------------|----------|
| 'execute_trajectory' | Clean, consistent with action file name | |
| 'movement_controller/execute_trajectory' | ROS2 convention of prefixing with package name | ✓ |
| No default, required parameter | Must be explicitly set in launch config | |

**User's choice:** `'movement_controller/execute_trajectory'`

**Notes:** `moveit_group_name` defaults to `'ur_manipulator'` (from ur_moveit_config SRDF — requirement UR-02).

---

## Goal Validation Boundary

| Option | Description | Selected |
|--------|-------------|----------|
| Validate in goal_callback — reject before accept | Pydantic validation runs in goal_callback; invalid goals get REJECT | ✓ |
| Validate in execute_callback — accept then abort | goal_callback always ACCEPT; invalid goals ABORTED with error result | |
| Split: lightweight pre-accept + deep Pydantic post-accept | Two-stage validation | |

**User's choice:** Validate in goal_callback

**Follow-up — What validation rules?**

| Option | Description | Selected |
|--------|-------------|----------|
| Structural only: non-empty list, valid motion_type, non-empty path_id | Basic structural checks | ✓ |
| Full business rules: blend radius logic, speed > 0, CIRC validity | All business rules validated early | |
| Field-level only: type constraints, enums, skip cross-path rules | Pydantic field constraints but not cross-path business logic | |

**User's choice:** Structural only in Phase 2 — cross-path rules deferred to Phase 3

**Follow-up — On validation failure?**

| Option | Description | Selected |
|--------|-------------|----------|
| Log + REJECT | Log error, return GoalResponse.REJECT | ✓ |
| Log + ACCEPT (fail in execution) | Log error, still ACCEPT, fail in execute_callback | |

**User's choice:** Log + REJECT

---

## Stub Execution Behavior

| Option | Description | Selected |
|--------|-------------|----------|
| Full feedback sequence: executing → completed → success | Mimics real contract, callers can build against it | ✓ |
| Immediate success, no feedback | Accept and immediately return success, no feedback | |
| executing feedback + NotImplementedError (explicit placeholder) | executing feedback then raises NotImplementedError | |

**User's choice:** Full feedback sequence per execution group

**Follow-up — trajectory_paths_completed content?**

| Option | Description | Selected |
|--------|-------------|----------|
| All path_ids echoed in trajectory_paths_completed | All path_ids from goal echoed in result | ✓ |
| Empty trajectory_paths_completed | Filled in Phase 3+ | |
| First path_id only | Simulates single-path execution | |

**User's choice:** All path_ids echoed in trajectory_paths_completed

**Follow-up — Feedback sequence count?**

**User's free-text clarification (critical):** 
> "Option 2 (per-path feedback pairs), but with an important clarification. TrajectoryPath objects with blend_radius > 0 must be executed as one single motion (using MoveGroupSequence under the hood) with the previous motion. The grouping algorithm: t[i] with blend_radius > 0 merges into the group containing t[i-1]. Example: [t0(br=0.5), t1(br=0), t2(br=0), t3(br=0.3), t4(br=0.3), t5(br=0.3), t6(br=0)] → groups [t0], [t1], [t2,t3,t4,t5], [t6] → 8 feedback messages."

**User's choice (clarified):** Per-group feedback pairs. Groups determined by blend_radius grouping algorithm.

**Follow-up — Blending condition (strict vs non-strict)?**

| Option | Description | Selected |
|--------|-------------|----------|
| blend_radius > 0 triggers blending | Strictly positive: 0.0 means no blending. Matches example. | ✓ |
| blend_radius >= 0 triggers blending | Even 0.0 triggers blending | |

**User's choice:** `blend_radius > 0` (strictly positive)

**Follow-up — Where does grouping logic live?**

| Option | Description | Selected |
|--------|-------------|----------|
| Grouping logic inline in execute_callback | Grouping code inside the callback | |
| Separate TrajectoryGrouper utility in utils/ | Reusable grouper utility | ✓ |
| Method on TrajectoryGoalDTO: get_execution_groups() | Encapsulated in model | |

**User's clarification:** Option 2 (TrajectoryGrouper in utils/), but grouping should happen in execute_callback before any execution logic.

**Follow-up — TrajectoryGrouper return type?**

| Option | Description | Selected |
|--------|-------------|----------|
| Returns list[list[TrajectoryPathDTO]] | Simple list-of-lists | ✓ |
| Returns list[TrajectoryGroupDTO] — named Pydantic model per group | Named model with is_blended, blend_radii | |
| Defer grouper design to Phase 3 | Inline grouping for now | |

**User's choice:** `list[list[TrajectoryPathDTO]]`

**Follow-up — Grouper validation?**

| Option | Description | Selected |
|--------|-------------|----------|
| Raise ValueError for bad input — caller handles | ValueError for duplicate path_ids, empty path_ids | ✓ |
| Return empty list for empty input, raise only for invalid values | Mixed | |
| No validation in grouper — pure grouping logic only | No validation | |

**User's clarification:** Raise ValueError for invalid inputs, BUT negative blend_radius is fine — it's silently treated as 0.0 (not an error).

---

## Concurrent Goal Rejection

| Option | Description | Selected |
|--------|-------------|----------|
| Reject in goal_callback — pre-accept, using an _is_executing flag | Fast rejection, client never gets a goal handle | ✓ |
| Accept then ABORT in execute_callback | Goal accepted, immediately returns ABORTED | |
| Use action server single-goal mode (framework handles it) | Let the framework handle concurrent rejection | |

**User's choice:** Reject in goal_callback with `_is_executing` flag

**User's additional clarification:** Action should ALSO reject any goals if node is in any other state than ACTIVE.

**Follow-up — Thread safety?**

| Option | Description | Selected |
|--------|-------------|----------|
| threading.Lock / threading.Event | Standard threading primitives | ✓ |
| asyncio.Lock | asyncio event loop | |
| No lock — flag only (single-thread executor assumed) | No synchronization | |

**User's choice:** `threading.Lock`

**Follow-up — When is _is_executing set?**

| Option | Description | Selected |
|--------|-------------|----------|
| While execute_callback is running only | Flag set at entry, cleared on exit of execute_callback | ✓ |
| From goal ACCEPT to result sent | Covers accept+execute window | |
| Only in ACTIVE state + executing | Combines lifecycle + execution flag | |

**User's clarification:** Option 1, plus action should reject any goals if node is not in ACTIVE state.

**Follow-up — Lifecycle state check implementation?**

| Option | Description | Selected |
|--------|-------------|----------|
| Use lifecycle_msgs.msg.State constants | `State.PRIMARY_STATE_ACTIVE` constant | ✓ |
| Compare state label string == 'active' | String comparison | |
| Store _is_active bool, set in on_activate / cleared in on_deactivate | Local bool flag | |

**User's choice:** `lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE`

---

## Agent's Discretion

- Specific log message formats and log levels (beyond ERROR for validation failures) — agent's discretion
- `TrajectoryPathDTO` exact field validators (beyond the minimum structural requirements specified)
- Internal structure of unit test fixtures and mock setup

## Deferred Ideas

- Cross-path business rule validation (blend radius logic, cartesian_speed > 0, CIRC intermediate point validity) — deferred to Phase 3 when planning makes these meaningful
- Action cancellation handling — deferred to Phase 3/4 when real execution makes cancellation meaningful
- `TrajectoryGoalDTO.get_execution_groups()` convenience method — discussed, not selected; `TrajectoryGrouper.group()` standalone utility chosen instead
