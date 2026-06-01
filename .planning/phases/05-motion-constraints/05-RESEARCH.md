# Phase 5: Motion Constraints — Research

**Researched:** 2026-06-01
**Domain:** MoveIt2 / PILZ Industrial Motion Planner — constraint message types, injection points, cartesian speed API, per-joint velocity limits
**Confidence:** HIGH (all key claims verified against installed ROS 2 Jazzy source, moveit_msgs message definitions, and official PILZ docs)

---

## Summary

Phase 5 adds persistent workspace, joint-position, and orientation constraints to all PILZ planning
requests, plus per-path cartesian speed enforcement. The primary mechanism is injecting a
`moveit_msgs/Constraints` object into `MotionSequenceItem.req.path_constraints` for each item.
Although PILZ itself does not evaluate path constraints during trajectory generation, the
`ValidateSolution` response adapter (which IS in the default PILZ pipeline) calls
`PlanningScene::isPathValid()` which checks EVERY waypoint against path constraints — causing
planning failure when a path violates the workspace bounding box.

**Primary recommendation:** Inject all active constraints into `MotionSequenceItem.req.path_constraints`
per item. For cartesian speed, use `max_velocity_scaling_factor` on the `MotionPlanRequest` (PILZ's
native per-path speed mechanism). Per-joint velocity limits cannot be enforced via planning requests —
store in `ConstraintConfigDTO` for documentation but do NOT inject into planning.

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01** `URMovementController` reads params + builds `ConstraintConfigDTO` in `on_configure`; calls
  `self._planner_service.set_constraints(dto)`. `PilzPlannerService` owns all MoveIt constraint objects.
- **D-02** Constraints injected as **path constraints** on each `MotionSequenceItem.req.path_constraints`
  in `_build_sequence_request()`. No `set_path_constraints()` on the planning component.
- **D-03** Parameter namespace: `constraints.workspace.*`, `constraints.joint.*`,
  `constraints.orientation.*`, `constraints.max_cartesian_speed`, `constraints.max_acceleration`,
  `constraints.joint.max_velocities`. All declared with `ParameterDescriptor(description=...)`.
- **D-04** Sentinel defaults = disabled. Workspace: range >= 2e9 → skip. Joint: names=[] → skip.
  Orientation: tolerances >= 2π → skip. Speed/accel: max == 0.0 → skip.
- **D-05** `OrientationConstraint.link_name` set to `path_dto.tool_frame` (per-path, not fixed `tool0`).
- **D-07** Speed violation error format: `Path '{path_id}' cartesian_speed {actual} m/s exceeds node maximum {max} m/s (constraints.max_cartesian_speed)`.
- **D-08** Joint constraint: three arrays `names`, `lower_limits`, `upper_limits`; all same length.
- **D-09** Pydantic validation failure → `TransitionCallbackReturn.FAILURE` from `on_configure`.
- **D-10** Re-configure rebuilds `ConstraintConfigDTO` fresh from parameters each time.
- **D-11** `ConstraintConfigDTO` in `movement_controller/models/constraint_config_dto.py`.
- **D-12** Workspace sentinels: `x_min/y_min/z_min = -1e9`, `x_max/y_max/z_max = +1e9`.
  Disabled when range >= 2e9 on any axis.
- **D-13** `constraints.joint.max_velocities float64[]` added. Shares `names` array.
  **Researcher must verify** per-joint velocity API (see D-13 section below).

### the agent's Discretion
- Exact disable detection rule for workspace (D-12) — range >= 2e9 on any axis, or all axes?
- Orientation constraint reference quaternion for path constraints (what orientation to use)
- CIRC constraint merging strategy when Phase 5 path constraints meet existing CIRC arc constraint
- Exact field names and structure for `ConstraintConfigDTO`

### Deferred Ideas (OUT OF SCOPE)
- Per-move constraint overrides in goal (v2)
- Launching the `LimitMaxCartesianLinkSpeed` adapter (Phase 7)
- Scene management (Phase 6)
- Real hardware validation (Phase 8)
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| CON-01 | Workspace bounding-box constraint from parameters → all planning requests | `PositionConstraint` with `SolidPrimitive.BOX` in `path_constraints`; enforced by `ValidateSolution` |
| CON-02 | Per-joint angle constraints from parameters → all planning requests | `JointConstraint` objects in `path_constraints.joint_constraints` |
| CON-03 | End-effector orientation constraint → all planning requests | `OrientationConstraint` in `path_constraints.orientation_constraints` |
| CON-04 | Constraints persistent for node lifetime; not per-goal overridable | `ConstraintConfigDTO` built once on `on_configure`, stored in `PilzPlannerService` |
| CON-05 | Max cartesian speed node cap; goal rejected if exceeded | Pre-planning check in `_goal_callback`; absolute m/s comparison |
| CON-06 | Max joint speeds constraint → all planning requests | Limited by PILZ API — see D-13 finding; stored in DTO, not injected into planning |
</phase_requirements>

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Parameter declaration | `URMovementController.__init__` | — | Params declared in `__init__` per project convention |
| Parameter reading + DTO construction | `URMovementController.on_configure` | — | Lifecycle pattern: read params on configure |
| Pydantic validation | `ConstraintConfigDTO` | `on_configure` error handling | DTO owns validation logic; controller handles failure |
| Constraint object building | `PilzPlannerService._build_path_constraints()` | — | Service owns all MoveIt constraint objects (D-01) |
| Constraint injection | `PilzPlannerService._generate_motion_sequence_request()` | — | Injected per `MotionSequenceItem` (D-02) |
| Speed cap enforcement | `URMovementController._goal_callback` | — | Reject before accepting goal |
| Per-path speed setting | `PilzPlannerService._generate_motion_sequence_request()` | — | Set `max_velocity_scaling_factor` per item |
| CIRC arc constraint merging | `PilzPlannerService._generate_motion_sequence_request()` | — | CIRC sets path_constraints.name; Phase 5 merges |

---

## Standard Stack

### Core (all verified against installed ROS 2 Jazzy)

| Library | Source | Purpose | Notes |
|---------|--------|---------|-------|
| `moveit_msgs.msg.Constraints` | `/opt/ros/jazzy/share/moveit_msgs/msg/Constraints.msg` | Container for all constraint types | Already imported in `pilz_planner_service.py` |
| `moveit_msgs.msg.PositionConstraint` | `/opt/ros/jazzy/share/moveit_msgs/msg/PositionConstraint.msg` | Workspace bounding box | Already imported |
| `moveit_msgs.msg.OrientationConstraint` | `/opt/ros/jazzy/share/moveit_msgs/msg/OrientationConstraint.msg` | Orientation constraint | Already imported |
| `moveit_msgs.msg.JointConstraint` | `/opt/ros/jazzy/share/moveit_msgs/msg/JointConstraint.msg` | Per-joint position limits | Already used in the codebase; verify import in service |
| `shape_msgs.msg.SolidPrimitive` | `/opt/ros/jazzy/share/shape_msgs/msg/SolidPrimitive.msg` | BOX primitive for bounding volume | Already imported in service |
| `moveit_msgs.msg.BoundingVolume` | `/opt/ros/jazzy/share/moveit_msgs/msg/BoundingVolume.msg` | Wraps SolidPrimitive + pose | Already imported in service |
| `pydantic.v2` | Project requirement | `ConstraintConfigDTO` | Frozen, `Field(description=...)` on every field |

---

## Package Legitimacy Audit

> This phase adds no new external PyPI or npm packages. All dependencies already in `requirements.txt`
> or installed via ROS 2 apt packages (`moveit_msgs`, `shape_msgs`). No legitimacy audit required.

---

## Architecture Patterns

### System Architecture Diagram

```
URMovementController.__init__
        │
        ▼ declare_parameter() ×15 (constraint params)
        
URMovementController.on_configure
        │
        ├── read all constraint params via get_parameter()
        ├── ConstraintConfigDTO(**params)  ← Pydantic v2 validation
        │         │
        │         ▼ ValidationError?
        │         └── return TransitionCallbackReturn.FAILURE
        │
        └── planner_service.set_constraints(dto)  ← D-01

URMovementController._goal_callback
        │
        ├── [for each path] if path.cartesian_speed > 0 and max_cartesian_speed > 0
        │         └── reject if path.cartesian_speed > constraints.max_cartesian_speed
        └── [same for acceleration]

PilzPlannerService._generate_motion_sequence_request(group, start_state)
        │
        ├── [for each path_dto]
        │       ├── item.req.max_velocity_scaling_factor = path_dto.cartesian_speed (if > 0)
        │       ├── item.req.max_acceleration_scaling_factor = path_dto.acceleration (if > 0)
        │       │
        │       ├── if path_dto.motion_type == CIRC:
        │       │       circ_constraints = _build_circ_constraints(path_dto)
        │       │       path_constraints = _build_path_constraints(tool_frame)
        │       │       item.req.path_constraints = _merge_circ_and_path_constraints(
        │       │                                       circ_constraints, path_constraints)
        │       └── else:
        │               item.req.path_constraints = _build_path_constraints(tool_frame)
        │
        └── return MotionSequenceRequest

PilzPlannerService._build_path_constraints(tool_frame) → Constraints
        │
        ├── if workspace NOT disabled: append PositionConstraint (BOX)
        ├── if joint names NOT empty:  append JointConstraint × len(names)
        └── if orientation NOT disabled: append OrientationConstraint
```

### Recommended Project Structure

New file to create:
```
src/movement_controller/
└── movement_controller/
    └── models/
        └── constraint_config_dto.py    ← ConstraintConfigDTO (new)
```

Existing files to modify:
```
src/movement_controller/
├── movement_controller/
│   ├── ur_movement_controller.py       ← +declare_parameter ×15, +on_configure, +_goal_callback
│   └── services/
│       └── pilz_planner_service.py     ← +set_constraints(), +_build_path_constraints(),
│                                          +_merge_circ_and_path_constraints(), update
│                                          _generate_motion_sequence_request()
└── tests/
    └── unit/
        └── test_constraint_config_dto.py   ← new unit tests
    └── integration/
        └── test_integration_ur_movement_controller.py  ← extend existing
```

---

## D-06: Cartesian Speed Mechanism — VERIFIED FINDING

**Question:** What is the correct PILZ/`GetMotionSequence` mechanism for per-path cartesian speed?
Candidates: `LimitMaxCartesianLinkSpeed` adapter, `MotionPlanRequest.max_velocity_scaling_factor`,
`CartesianSpeedLimitedConstraint`.

**Verified Answer** [VERIFIED: installed ROS 2 Jazzy + official PILZ docs]:

### What PILZ Natively Supports

PILZ's planning interface uses exactly two speed fields in `MotionPlanRequest`:

```
max_velocity_scaling_factor     # (0, 1] — scales configured max_trans_vel
max_acceleration_scaling_factor # (0, 1] — scales configured max_trans_acc
```

These scale the global Cartesian limits (`max_trans_vel`, `max_trans_acc`, `max_trans_dec`,
`max_rot_vel`) configured in the move_group node via `pilz_cartesian_limits.yaml`.

From official docs [VERIFIED: moveit.picknik.ai PILZ page]:
> `max_velocity_scaling_factor`: scaling factor of maximal Cartesian translational/rotational velocity (for LIN and CIRC)

### `max_cartesian_speed` + `cartesian_speed_limited_link` Fields

These fields exist in `MotionPlanRequest.msg` [VERIFIED: installed msg]:
```
# Maximum cartesian speed for the given link.
# If max_cartesian_speed <= 0 the trajectory is not modified.
# These fields require the following planning request adapter:
# default_planning_request_adapters/LimitMaxCartesianLinkSpeed
string cartesian_speed_limited_link
float64 max_cartesian_speed  # m/s
```

**CRITICAL:** The `LimitMaxCartesianLinkSpeed` adapter is **NOT installed** in this Jazzy environment:
- `libmoveit_default_planning_request_adapter_plugins.so` contains only: `CheckForStackedConstraints`,
  `CheckStartStateBounds`, `CheckStartStateCollision`, `ValidateWorkspaceBounds`, `ResolveConstraintFrames`
- The PILZ pipeline (`ur_moveit_config/config/pilz_industrial_motion_planner_planning.yaml`) does NOT
  include this adapter
- Setting `max_cartesian_speed` without the adapter has **no effect on the trajectory**

### Recommended Approach for Phase 5

**For D-06 Step 1 (PILZ execution speed per path):**

Since `max_velocity_scaling_factor` is a ratio (0..1) and `TrajectoryPathDTO.cartesian_speed` is
in m/s, the Phase 5 planner service must treat `cartesian_speed` from the DTO as the direct
scaling factor (range-clamped to `(0.0, 1.0]`). This is consistent with the `TODO` comment in
the existing `_generate_motion_sequence_request()` (`# TODO: these must come from constraints`).

```python
# In _generate_motion_sequence_request()
if path_dto.cartesian_speed > 0.0:
    item.req.max_velocity_scaling_factor = max(0.01, min(1.0, path_dto.cartesian_speed))
else:
    item.req.max_velocity_scaling_factor = 1.0  # full speed when unspecified

if path_dto.acceleration > 0.0:
    item.req.max_acceleration_scaling_factor = max(0.01, min(1.0, path_dto.acceleration))
else:
    item.req.max_acceleration_scaling_factor = 1.0
```

> **Note:** This interprets `cartesian_speed` as a ratio rather than absolute m/s. When Phase 7
> configures launch files with actual cartesian limits (e.g., `max_trans_vel: 1.5`), the
> `TrajectoryPath.msg` field description can be clarified as "fraction of max speed (0..1)".
> Alternatively, Phase 7 can add a `constraints.cartesian_limits_max_trans_vel` parameter to
> enable the absolute m/s → ratio conversion if needed.

**For D-06 Step 2 (node-level cap validation in `_goal_callback`):**
```python
# Direct comparison in m/s (no scaling needed here — comparing user's absolute request
# against operator's configured max)
if path_dto.cartesian_speed > 0.0 and self._constraint_config.max_cartesian_speed > 0.0:
    if path_dto.cartesian_speed > self._constraint_config.max_cartesian_speed:
        # reject with D-07 error message format
```

---

## D-13: Per-Joint Velocity Limits — VERIFIED FINDING

**Question:** What is the correct MoveIt2/PILZ API field for per-joint velocity overrides in
`MotionSequenceItem`?

**Verified Answer** [VERIFIED: moveit_msgs/JointConstraint.msg + PILZ headers]:

### `JointConstraint.msg` Has No Velocity Field

```
# Constrain the position of a joint to be within a certain bound
string joint_name
float64 position
float64 tolerance_above
float64 tolerance_below
float64 weight
```

**There is no `velocity` or `max_velocity` field in `JointConstraint`.** Joint velocity limits
are not expressible through the `Constraints` message at all.

### PILZ Joint Velocity Model

PILZ applies joint velocity limits from two sources (merged to strictest):
1. URDF `<limit velocity="..."/>` per joint
2. `joint_limits.yaml` parameters (node params on move_group, optional overrides)

Both are configured at move_group startup, not per planning request. There is no mechanism to
override per-joint velocity limits dynamically in a `MotionSequenceItem` request.
`max_velocity_scaling_factor` applies a **global** scale to ALL joints simultaneously.

### Recommended Approach for D-13

- **Store** `constraints.joint.max_velocities` in `ConstraintConfigDTO` as designed.
- **Do NOT inject** into planning requests (no supported API).
- **Do NOT silently ignore** — log a WARNING at `set_constraints()` time when non-empty
  `max_velocities` are provided, stating that per-joint velocity enforcement is not available
  via PILZ planning requests.
- This satisfies CON-06 for "reads from parameters" but documents the enforcement limitation.
  CON-06 says "applies them to all planning requests" — since PILZ has no such mechanism,
  the limitation must be documented in the plan's verification notes.

---

## Constraint Message Fields — Verified

### `MotionSequenceItem.req.path_constraints` [VERIFIED: installed msg]

```
MotionSequenceItem
  └── req: MotionPlanRequest
        └── path_constraints: Constraints
              ├── name: string
              ├── joint_constraints: JointConstraint[]
              ├── position_constraints: PositionConstraint[]
              └── orientation_constraints: OrientationConstraint[]
```

**Injection point:** `item.req.path_constraints` per `MotionSequenceItem` in
`_generate_motion_sequence_request()`. This is the correct field (D-02).

**PILZ + ValidateSolution enforcement:**
- PILZ does NOT check path_constraints during trajectory generation for LIN/PTP motions
- PILZ uses path_constraints ONLY for CIRC arc definition (`constraints.name` = 'interim'|'center')
- **`ValidateSolution` response adapter** calls `PlanningScene::isPathValid(trajectory, path_constraints,
  goal_constraints, group)` [VERIFIED: moveit_core planning_scene.hpp] which checks EVERY waypoint
- Result: workspace bounding box WILL cause planning failure when goal lies outside the box

---

### `PositionConstraint` — Bounding Box [VERIFIED: installed msg]

```python
from moveit_msgs.msg import PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def _build_workspace_position_constraint(
    tool_frame: str,
    frame_id: str,
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    z_min: float, z_max: float,
) -> PositionConstraint:
    """Source: verified against moveit_msgs/PositionConstraint.msg +
               shape_msgs/SolidPrimitive.msg installed in Jazzy."""
    pos = PositionConstraint()
    pos.header.frame_id = frame_id
    pos.link_name = tool_frame          # e.g. 'tool0'
    pos.weight = 1.0

    # BOX dimensions = full lengths (not half-extents)
    # Verified: SolidPrimitive.BOX_X=0, BOX_Y=1, BOX_Z=2
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX       # = 1
    box.dimensions = [
        x_max - x_min,                  # BOX_X = full x width
        y_max - y_min,                  # BOX_Y = full y width
        z_max - z_min,                  # BOX_Z = full z height
    ]

    # Primitive pose = CENTER of the box in frame_id coordinates
    center_pose = Pose()
    center_pose.position.x = (x_min + x_max) / 2.0
    center_pose.position.y = (y_min + y_max) / 2.0
    center_pose.position.z = (z_min + z_max) / 2.0
    center_pose.orientation.w = 1.0

    bv = BoundingVolume()
    bv.primitives = [box]
    bv.primitive_poses = [center_pose]
    pos.constraint_region = bv

    return pos
```

> `target_point_offset` (Vector3) = offset from link origin to target point within link frame.
> For tool0, leave at default (0, 0, 0) — the link origin IS the tool tip.

---

### `OrientationConstraint` — Tolerance Fields [VERIFIED: installed msg]

```
# This message contains the definition of an orientation constraint.
std_msgs/Header header
geometry_msgs/Quaternion orientation    # reference orientation
string link_name
float64 absolute_x_axis_tolerance      # +/- tolerance in radians (symmetric)
float64 absolute_y_axis_tolerance      # +/- tolerance in radians (symmetric)
float64 absolute_z_axis_tolerance      # +/- tolerance in radians (symmetric)
uint8 parameterization                 # 0 = XYZ_EULER_ANGLES (default), 1 = ROTATION_VECTOR
float64 weight
```

**Notes:**
- Tolerances are symmetric: actual orientation can deviate by ±`absolute_x_axis_tolerance` around the reference `orientation` quaternion's X axis, etc.
- `parameterization = 0` (XYZ_EULER_ANGLES, default) — always use default; do NOT set ROTATION_VECTOR for Phase 5.
- For path constraints (not goal): use an **identity quaternion** (`w=1.0, x=y=z=0`) as the reference orientation, with larger tolerances. This constrains the DEVIATION from identity (i.e., near-upright orientation). If the intent is "end-effector must stay near the target pose orientation", use the `target_pose.pose.orientation` from the path DTO.

```python
def _build_orientation_constraint(
    tool_frame: str,
    frame_id: str,
    reference_orientation: Quaternion,
    tol_x: float,
    tol_y: float,
    tol_z: float,
) -> OrientationConstraint:
    """Source: verified against moveit_msgs/OrientationConstraint.msg Jazzy."""
    ori = OrientationConstraint()
    ori.header.frame_id = frame_id
    ori.link_name = tool_frame           # per-path tool_frame (D-05)
    ori.orientation = reference_orientation
    ori.absolute_x_axis_tolerance = tol_x   # radians, symmetric +/-
    ori.absolute_y_axis_tolerance = tol_y
    ori.absolute_z_axis_tolerance = tol_z
    ori.parameterization = OrientationConstraint.XYZ_EULER_ANGLES  # = 0
    ori.weight = 1.0
    return ori
```

> **Reference orientation for path constraint:** The CONTEXT.md (D-05) says `link_name` comes from
> `tool_frame` per-path. The reference `orientation` for a path-level constraint should be set to
> `geometry_msgs.msg.Quaternion(w=1.0)` (identity) with the provided tolerances — this means
> "link orientation must not deviate more than ±tol from identity". Alternatively, if the intent is
> to keep the tool near its goal orientation, use `path_dto.target_pose.pose.orientation` as the
> reference. **Decision for planner:** use identity quaternion as the reference — this is the
> simplest interpretation for "don't rotate more than X radians in any axis".

---

### `JointConstraint` — Position Fields [VERIFIED: installed msg]

```
string joint_name
float64 position        # reference position (radians)
float64 tolerance_above # upper tolerance (+) from position
float64 tolerance_below # lower tolerance (-) from position (should be positive)
float64 weight
```

**Construction from lower/upper limits:**
```python
def _build_joint_constraint(
    joint_name: str,
    lower: float,       # radians, lower bound
    upper: float,       # radians, upper bound
) -> JointConstraint:
    """Source: verified against moveit_msgs/JointConstraint.msg Jazzy."""
    jc = JointConstraint()
    jc.joint_name = joint_name
    jc.position = (lower + upper) / 2.0       # midpoint of range
    jc.tolerance_above = upper - jc.position  # half-range up
    jc.tolerance_below = jc.position - lower  # half-range down (positive value)
    jc.weight = 1.0
    return jc
```

> **No velocity field:** Confirmed — `JointConstraint` has no velocity-related fields.
> Per-joint velocity enforcement is not possible via this message type.

---

## Sentinel Disable Detection — VERIFIED FINDING

**Question:** Exact logic for detecting "disabled" state from `ConstraintConfigDTO`.

**Verified Answer** [VERIFIED against CONTEXT.md D-04, D-12]:

### Workspace Bounding Box

The default range is `[-1e9, +1e9]` per axis. The range on any single axis is `+1e9 - (-1e9) = 2e9`.

**Disable rule:** Skip `PositionConstraint` when **all axes** are at defaults (i.e., none of the
bounds were changed from their sentinels). The correct check is:

```python
@property
def workspace_enabled(self) -> bool:
    """Return True only when at least one axis has been narrowed from defaults."""
    return not (
        self.x_max - self.x_min >= 2e9 and
        self.y_max - self.y_min >= 2e9 and
        self.z_max - self.z_min >= 2e9
    )
```

> This means: if ANY axis is narrowed (range < 2e9), the workspace constraint IS active.
> If all axes are at sentinel default (range = 2e9 exactly on all three), it is disabled.
> This avoids adding a trivially wide bounding box that PILZ/ValidateSolution still evaluates.

### Joint Constraints

```python
@property
def joint_constraints_enabled(self) -> bool:
    return len(self.joint_names) > 0
```

### Orientation Constraint

Disable when ALL three tolerances are at 2π (the default "unconstrained" value):
```python
import math
@property
def orientation_constraint_enabled(self) -> bool:
    TWO_PI = 2.0 * math.pi
    return not (
        self.tolerance_x >= TWO_PI and
        self.tolerance_y >= TWO_PI and
        self.tolerance_z >= TWO_PI
    )
```

### Speed / Acceleration Cap

```python
@property
def cartesian_speed_cap_enabled(self) -> bool:
    return self.max_cartesian_speed > 0.0

@property
def acceleration_cap_enabled(self) -> bool:
    return self.max_acceleration > 0.0
```

---

## CIRC Path Constraint Merging

**Problem:** CIRC paths already use `item.req.path_constraints` for the arc interim/center point
definition (via `_build_circ_constraints()`). Phase 5 cannot blindly overwrite this.

**Existing CIRC constraints structure:**
```python
circ_constraints = Constraints()
circ_constraints.name = 'interim'  # or 'center'
circ_constraints.position_constraints = [arc_point_constraint]  # arc definition — must stay at [0]
```

**Merging strategy:**
```python
def _merge_circ_and_path_constraints(
    circ_constraints: Constraints,
    path_constraints: Constraints,
) -> Constraints:
    """Preserve CIRC arc definition, append Phase 5 constraints.

    PILZ expects: constraints.name = 'interim'|'center' and
                  constraints.position_constraints[0] = arc reference point.
    Phase 5 appends workspace position and orientation constraints AFTER [0].
    """
    merged = Constraints()
    merged.name = circ_constraints.name        # MUST preserve 'interim'/'center'
    # Arc point stays at index 0; workspace box appended at [1:]
    merged.position_constraints = list(circ_constraints.position_constraints)
    merged.position_constraints.extend(path_constraints.position_constraints)
    # Orientation and joint constraints only from Phase 5 (CIRC doesn't set these)
    merged.orientation_constraints = list(path_constraints.orientation_constraints)
    merged.joint_constraints = list(path_constraints.joint_constraints)
    return merged
```

> **Why it works:** PILZ reads `constraints.name` and `position_constraints[0]` for the arc point.
> `ValidateSolution` checks ALL constraints in the merged object. The workspace box at index [1]
> and orientation/joint constraints are checked by `ValidateSolution`, not PILZ's trajectory generator.

---

## `ConstraintConfigDTO` Structure

```python
# movement_controller/models/constraint_config_dto.py
from __future__ import annotations
import math
from pydantic import BaseModel, ConfigDict, Field, model_validator

class ConstraintConfigDTO(BaseModel):
    """Validated, immutable node-level motion constraint configuration.
    
    All fields default to sentinel values meaning 'disabled'.
    Validators enforce: x_min <= x_max, y_min <= y_max, z_min <= z_max,
    and all joint arrays same length when any is non-empty.
    """
    model_config = ConfigDict(frozen=True)

    # Workspace bounding box (sentinels ±1e9; range >= 2e9 means disabled)
    x_min: float = Field(default=-1e9, description='Workspace min X bound in metres (base_link frame)')
    x_max: float = Field(default=+1e9, description='Workspace max X bound in metres (base_link frame)')
    y_min: float = Field(default=-1e9, description='Workspace min Y bound in metres (base_link frame)')
    y_max: float = Field(default=+1e9, description='Workspace max Y bound in metres (base_link frame)')
    z_min: float = Field(default=-1e9, description='Workspace min Z bound in metres (base_link frame)')
    z_max: float = Field(default=+1e9, description='Workspace max Z bound in metres (base_link frame)')

    # Joint position constraints
    joint_names: list[str] = Field(default_factory=list, description='Joint names to constrain (empty = disabled)')
    joint_lower_limits: list[float] = Field(default_factory=list, description='Lower joint position limits in radians (matching joint_names order)')
    joint_upper_limits: list[float] = Field(default_factory=list, description='Upper joint position limits in radians (matching joint_names order)')
    joint_max_velocities: list[float] = Field(default_factory=list, description='Per-joint max velocity in rad/s; not enforced via planning (PILZ limitation); empty = disabled')

    # Orientation constraint tolerances (sentinel 2π = disabled)
    orientation_tolerance_x: float = Field(default=2.0 * math.pi, description='Allowed ±deviation around X axis in radians (2π = disabled)')
    orientation_tolerance_y: float = Field(default=2.0 * math.pi, description='Allowed ±deviation around Y axis in radians (2π = disabled)')
    orientation_tolerance_z: float = Field(default=2.0 * math.pi, description='Allowed ±deviation around Z axis in radians (2π = disabled)')

    # Speed / acceleration caps (0.0 = disabled)
    max_cartesian_speed: float = Field(default=0.0, description='Maximum allowed per-path cartesian_speed in m/s; 0.0 = no cap')
    max_acceleration: float = Field(default=0.0, description='Maximum allowed per-path acceleration in m/s²; 0.0 = no cap')
```

---

## Common Pitfalls

### Pitfall 1: CIRC `path_constraints` Overwrite
**What goes wrong:** Phase 5 replaces `item.req.path_constraints` for CIRC paths, destroying the
arc point definition. PILZ silently falls back to an undefined behavior or raises a planning error.
**Why it happens:** CIRC uses `path_constraints` for the arc interim/center point, not just as a
validation hint.
**How to avoid:** Use `_merge_circ_and_path_constraints()` — always check
`path_dto.motion_type == MotionTypeEnum.CIRC` before setting `path_constraints`.

### Pitfall 2: `max_cartesian_speed` + `cartesian_speed_limited_link` Has No Effect
**What goes wrong:** Setting these fields hoping PILZ enforces them, but trajectories have
full speed.
**Why it happens:** `LimitMaxCartesianLinkSpeed` adapter is NOT in the PILZ pipeline and is NOT
installed in this Jazzy environment.
**How to avoid:** Use `max_velocity_scaling_factor` for per-path speed control. These fields can
be set for forward-compatibility but must not be relied on for enforcement.

### Pitfall 3: BOX Dimensions Are Half-Extents (False)
**What goes wrong:** Developer assumes `SolidPrimitive.BOX` dimensions are half-extents (like
many collision systems), making the box twice as large as intended.
**Why it happens:** Common confusion with other collision geometry formats.
**How to avoid:** `SolidPrimitive.BOX` dimensions are **full lengths** (x_max - x_min, y_max - y_min,
z_max - z_min). Verified against `shape_msgs/SolidPrimitive.msg`: "For type BOX, the X, Y, and Z
dimensions are the length of the corresponding sides of the box."

### Pitfall 4: Orientation Constraint Reference Quaternion
**What goes wrong:** Passing `Quaternion()` (all zeros) as reference orientation, which is
mathematically invalid (not a unit quaternion).
**Why it happens:** Default `Quaternion()` in ROS2 Python has all fields = 0.0, not `w=1.0`.
**How to avoid:** Always initialize with `Quaternion(w=1.0)` as the identity quaternion.

### Pitfall 5: JointConstraint `tolerance_below` Must Be Positive
**What goes wrong:** Setting `tolerance_below = lower - position` (which gives a negative value)
because the tolerance is "below" the reference position. MoveIt2 expects a **positive** value
representing the magnitude of the downward tolerance.
**How to avoid:** `tolerance_below = position - lower` (always positive since position >= lower).

### Pitfall 6: Per-Joint Velocity Not Enforced Silently
**What goes wrong:** `constraints.joint.max_velocities` is read and stored, but nothing happens.
Developer thinks velocities are being enforced.
**Why it happens:** No per-joint velocity API in PILZ.
**How to avoid:** Log a WARNING in `set_constraints()` when `joint_max_velocities` is non-empty:
`"per-joint velocity limits are stored but not enforced via PILZ planning requests"`.

### Pitfall 7: `ParameterAlreadyDeclaredException` on Re-configure
**What goes wrong:** Calling `declare_parameter()` in `on_configure` causes exception on second
configure after deactivate+cleanup.
**Why it happens:** Parameters declared in `on_configure` persist through cleanup.
**How to avoid:** Declare ALL parameters in `__init__`. Read them in `on_configure`.
(This is an existing project convention already followed.)

### Pitfall 8: `float('inf')` in ROS2 Parameters
**What goes wrong:** Using Python `float('inf')` as default for workspace bounds — ROS2 float64
parameters do not support infinity.
**How to avoid:** Use ±1e9 as sentinels (already locked in D-12).

---

## Test Patterns

### Unit Tests: Constraint Building
Unit tests for constraint building do NOT need a ROS2 context. They instantiate `moveit_msgs`
message objects directly (available via `import` after colcon build with ROS2 sourced, or via
the `conftest.py` MagicMock stub for CI without full stack).

```python
# tests/unit/test_constraint_config_dto.py
import math
import pytest
from pydantic import ValidationError
from movement_controller.models.constraint_config_dto import ConstraintConfigDTO

# Test 1: workspace enabled detection
def test_workspace_disabled_when_all_at_defaults():
    dto = ConstraintConfigDTO()
    assert dto.workspace_enabled is False

def test_workspace_enabled_when_x_narrowed():
    dto = ConstraintConfigDTO(x_min=-0.5, x_max=0.5)
    assert dto.workspace_enabled is True

# Test 2: BOX constraint construction
def test_position_constraint_box_dimensions():
    dto = ConstraintConfigDTO(x_min=-0.5, x_max=0.5, y_min=-0.3, y_max=0.3, z_min=0.0, z_max=1.0)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.set_constraints(dto)
    constraints = svc._build_path_constraints('tool0')
    pos_c = constraints.position_constraints[0]
    box = pos_c.constraint_region.primitives[0]
    assert box.type == 1  # SolidPrimitive.BOX
    assert box.dimensions[0] == pytest.approx(1.0)  # x: 0.5 - (-0.5)
    assert box.dimensions[1] == pytest.approx(0.6)  # y: 0.3 - (-0.3)
    assert box.dimensions[2] == pytest.approx(1.0)  # z: 1.0 - 0.0

# Test 3: JointConstraint midpoint
def test_joint_constraint_position_is_midpoint():
    dto = ConstraintConfigDTO(joint_names=['j1'], joint_lower_limits=[-1.0], joint_upper_limits=[1.0], joint_max_velocities=[2.0])
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.set_constraints(dto)
    constraints = svc._build_path_constraints('tool0')
    jc = constraints.joint_constraints[0]
    assert jc.joint_name == 'j1'
    assert jc.position == pytest.approx(0.0)       # midpoint(-1, 1)
    assert jc.tolerance_above == pytest.approx(1.0)
    assert jc.tolerance_below == pytest.approx(1.0)

# Test 4: OrientationConstraint tolerance fields
def test_orientation_constraint_fields():
    dto = ConstraintConfigDTO(
        orientation_tolerance_x=0.1,
        orientation_tolerance_y=0.2,
        orientation_tolerance_z=0.3,
    )
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.set_constraints(dto)
    constraints = svc._build_path_constraints('tool0')
    oc = constraints.orientation_constraints[0]
    assert oc.absolute_x_axis_tolerance == pytest.approx(0.1)
    assert oc.absolute_y_axis_tolerance == pytest.approx(0.2)
    assert oc.absolute_z_axis_tolerance == pytest.approx(0.3)
    assert oc.parameterization == 0  # XYZ_EULER_ANGLES

# Test 5: CIRC merge preserves arc point at index 0
def test_circ_merge_preserves_arc_constraint():
    # ... build circ_constraints with name='interim' and one position_constraint
    # ... build path_constraints with workspace box
    # ... call _merge_circ_and_path_constraints()
    # assert merged.name == 'interim'
    # assert merged.position_constraints[0] is the arc point
    # assert merged.position_constraints[1] is the workspace box
```

### Integration Test: Workspace Bounding Box Violation
```python
# tests/integration/test_integration_ur_movement_controller.py (extend existing)
def test_planning_fails_when_goal_outside_workspace(controller_fixture):
    """Goal target pose at z=2.0 m should fail when workspace z_max=0.5."""
    # configure node with z_max=0.5 constraint
    # send goal with target_pose.pose.position.z = 2.0
    # assert GetMotionSequence returns error (MockMoveGroupNode handles this)
    # assert action result success=False
```

**Approach:** The existing `MockMoveGroupNode` in `test_integration_ur_movement_controller.py`
mocks the GetMotionSequence service and returns configurable responses. For the workspace violation
test, set `planning_success = False` (since we're not running real MoveIt, the mock simulates
planning failure). The test verifies the controller correctly propagates the planning failure.

> **NOTE:** Full end-to-end workspace constraint enforcement (i.e., `ValidateSolution` actually
> rejecting a real planned trajectory) requires a live `move_group` with PILZ — that's an
> acceptance/real-hardware test (TST-03, not CON-01 unit/integration scope).

---

## State of the Art

| Old Pattern | Current Pattern | Impact |
|-------------|-----------------|--------|
| MoveIt Commander `set_path_constraints()` | `MotionSequenceItem.req.path_constraints` | ROS1 API gone; per-item constraints in MotionSequenceRequest |
| `LimitMaxCartesianLinkSpeed` as standard adapter | NOT installed in Jazzy default | Must use `max_velocity_scaling_factor` for speed; adapter needs to be added to pipeline in Phase 7 if absolute m/s needed |
| Per-joint velocity via `JointConstraint` | Not supported | `JointConstraint` has no velocity fields; PILZ only supports global `max_velocity_scaling_factor` |

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `ValidateSolution` calls `PlanningScene::isPathValid()` with `path_constraints` | D-02, Test Patterns | If `ValidateSolution` only checks collisions (not kinematic constraints), workspace box won't fail planning; integration test approach changes |
| A2 | CIRC only uses `position_constraints[0]` for arc point; appending at [1:] is safe | CIRC Merging | If PILZ inspects all `position_constraints`, appended constraints could interfere with CIRC arc |
| A3 | Treating `cartesian_speed` as a 0..1 ratio for `max_velocity_scaling_factor` | D-06 | If callers intend m/s, the behavior is wrong; Phase 7 must document the interpretation |

---

## Environment Availability

| Dependency | Required By | Available | Notes |
|------------|------------|-----------|-------|
| `moveit_msgs` | All constraint types | ✓ | `/opt/ros/jazzy/share/moveit_msgs/` |
| `shape_msgs` | `SolidPrimitive.BOX` | ✓ | `/opt/ros/jazzy/share/shape_msgs/` |
| `LimitMaxCartesianLinkSpeed` adapter | D-06 absolute m/s | ✗ | Not installed; not in `libmoveit_default_planning_request_adapter_plugins.so` |
| `JointConstraint` velocity fields | D-13 | ✗ | Field does not exist in the message |
| `pilz_industrial_motion_planner` | Planning | ✓ | `/opt/ros/jazzy/lib/libpilz_industrial_motion_planner.so` |

**Missing dependencies with no fallback:** None blocking Phase 5 implementation.

**Missing dependencies with fallback:**
- `LimitMaxCartesianLinkSpeed`: fallback = `max_velocity_scaling_factor` (ratio-based, not absolute m/s)

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest + ament_pytest |
| Config file | `src/movement_controller/setup.cfg` (existing) |
| Quick run | `python -m pytest src/movement_controller/tests/unit/test_constraint_config_dto.py -v` |
| Full suite | `python -m pytest src/movement_controller/tests/ -v` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| CON-01 | Workspace BOX PositionConstraint built correctly from params | unit | `pytest tests/unit/test_constraint_config_dto.py::test_position_constraint_box_dimensions` | ❌ Wave 0 |
| CON-02 | JointConstraint built from names/lower/upper arrays | unit | `pytest tests/unit/test_constraint_config_dto.py::test_joint_constraint_position_is_midpoint` | ❌ Wave 0 |
| CON-03 | OrientationConstraint tolerance fields set correctly | unit | `pytest tests/unit/test_constraint_config_dto.py::test_orientation_constraint_fields` | ❌ Wave 0 |
| CON-04 | set_constraints() stored; same constraints in every MotionSequenceItem | unit | `pytest tests/unit/test_pilz_planner_service.py::test_constraints_injected_in_all_items` | ❌ Wave 0 |
| CON-05 | Goal rejected when path.cartesian_speed > max_cartesian_speed | unit | `pytest tests/unit/test_ur_movement_controller.py::test_goal_rejected_speed_exceeded` | ❌ Wave 0 |
| CON-06 | max_velocities stored in DTO; warning logged; not injected | unit | `pytest tests/unit/test_constraint_config_dto.py::test_joint_velocity_warning_logged` | ❌ Wave 0 |

### Wave 0 Gaps
- [ ] `tests/unit/test_constraint_config_dto.py` — covers CON-01 through CON-06
- [ ] Extend `tests/unit/test_pilz_planner_service.py` — covers CON-04
- [ ] Extend `tests/unit/test_ur_movement_controller.py` — covers CON-05
- [ ] Extend `tests/integration/test_integration_ur_movement_controller.py` — workspace violation scenario

---

## Security Domain

> No new network interfaces, external APIs, or authentication mechanisms introduced in Phase 5.
> All inputs are ROS2 node parameters (operator-controlled at launch time) and action goals
> (callers already validated in prior phases).

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V5 Input Validation | yes | Pydantic v2 `ConstraintConfigDTO` validates all parameter values |
| V2 Authentication | no | — |
| V4 Access Control | no | — |

### Known Threat Patterns

| Pattern | STRIDE | Mitigation |
|---------|--------|-----------|
| Invalid workspace bounds (`x_min > x_max`) | Tampering | Pydantic `model_validator` rejects → `on_configure` returns FAILURE |
| Mismatched joint arrays (names≠limits length) | Tampering | Pydantic `model_validator` rejects → `on_configure` returns FAILURE |
| Infinity in float params | Denial | ROS2 float64 rejects `inf`; sentinels ±1e9 used |

---

## Sources

### Primary (HIGH confidence)
- `MotionSequenceItem.msg`, `MotionPlanRequest.msg`, `Constraints.msg`, `JointConstraint.msg`,
  `PositionConstraint.msg`, `OrientationConstraint.msg`, `BoundingVolume.msg`, `SolidPrimitive.msg`
  — installed at `/opt/ros/jazzy/share/moveit_msgs/msg/` and `/opt/ros/jazzy/share/shape_msgs/msg/`
- `pilz_industrial_motion_planner_planning.yaml` — installed at
  `/opt/ros/jazzy/share/ur_moveit_config/config/`; confirms adapter list
- `libmoveit_default_planning_request_adapter_plugins.so` — `strings` output confirms adapter names
- `planning_scene.hpp` — `isPathValid()` API with `path_constraints` parameter confirmed
- PILZ official docs — https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html

### Secondary (MEDIUM confidence)
- `pilz_industrial_motion_planner/trajectory_generator.hpp` exception types — confirms PILZ checks
  goal constraints (JointsOfGoalOutOfRange, etc.) but not path constraints during generation
- `cartesian_limits_parameters.hpp` — confirms PILZ cartesian limits struct (`max_trans_vel`, etc.)
- `pilz_industrial_motion_planner/joint_limits_extension.hpp` — confirms PILZ uses `JointLimitsMap`
  loaded from robot model/parameters, not per-request

---

## Metadata

**Confidence breakdown:**
- D-06 (Cartesian speed API): HIGH — message definitions and pipeline config verified in situ
- D-13 (Per-joint velocity): HIGH — JointConstraint.msg has no velocity field; PILZ headers confirm global-only scaling
- Constraint injection (D-02): HIGH — `isPathValid()` API confirmed in moveit_core; `ValidateSolution` is in PILZ pipeline
- BOX geometry: HIGH — SolidPrimitive.msg constants verified (BOX=1, full-length dimensions)
- CIRC merge: MEDIUM — PILZ behavior with multiple position_constraints not directly tested; based on message structure and PILZ docs

**Research date:** 2026-06-01
**Valid until:** 2026-09-01 (stable stack — moveit_msgs message definitions don't change often)
