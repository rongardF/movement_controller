# Phase 2: LifecycleNode & Action Server Skeleton — Research

**Researched:** 2026-05-27
**Domain:** ROS2 Jazzy — `rclpy.lifecycle`, `rclpy.action`, Pydantic v2 DTOs
**Confidence:** HIGH (all APIs verified against installed rclpy 0.x in the devcontainer)

---

## Summary

Phase 2 implements a `rclpy.lifecycle.LifecycleNode` subclass (`URMovementController`) with a fully wired `ExecuteTrajectory` action server, Pydantic v2 data models, and a `TrajectoryGrouper` utility. No MoveIt2 is involved.

All critical rclpy APIs were verified live against the Jazzy installation in the devcontainer. The most important finding is a **discrepancy from the CONTEXT.md assumption**: `self.get_current_state()` **does not exist** in rclpy Jazzy — the correct API to check lifecycle state in `goal_callback` is `self._state_machine.current_state[0]`. This must be reflected in the plan.

All Phase 1 interface files were read directly from source and fields confirmed. Pydantic 2.13.4 is installed. No new packages need to be installed for this phase.

**Primary recommendation:** Implement lifecycle state checking via `self._state_machine.current_state[0] != State.PRIMARY_STATE_ACTIVE`. Use `TransitionCallbackReturn.SUCCESS` as the return from lifecycle callbacks. Wire `ActionServer` in `on_configure`; add `ament_add_pytest_test` entries to CMakeLists.txt for each new test file.

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **D-01:** `on_configure` declares exactly two parameters: `action_server_name` (string, default `'movement_controller/execute_trajectory'`) and `moveit_group_name` (string, default `'ur_manipulator'`).
- **D-02:** Validation in `goal_callback` — structural check, REJECT before accept.
- **D-03:** Phase 2 validation rules: non-empty `paths` list; each path's `motion_type` is one of `"LIN"`, `"PTP"`, `"CIRC"`; each `path_id` is non-empty string. Speed/CIRC-specific rules deferred to Phase 3.
- **D-04:** On validation failure: log at ERROR level, return `GoalResponse.REJECT`.
- **D-05:** `execute_callback` sends full per-group feedback sequence (no actual planning).
- **D-06:** Feedback sequence per group: `{status: 'executing', trajectory_path_ids: [all ids in group]}` then `{status: 'completed', trajectory_path_ids: [all ids in group]}`. Result: `trajectory_paths_completed` = all path_ids.
- **D-07:** Blend grouping algorithm: first path always starts new group; `blend_radius > 0` (and not first) merges into current group; `blend_radius <= 0` starts new group; negative `blend_radius` silently treated as `0.0`.
- **D-08:** `TrajectoryGrouper` is a standalone class in `movement_controller/utils/trajectory_grouper.py`.
- **D-09:** Interface: `TrajectoryGrouper.group(paths: list[TrajectoryPathDTO]) -> list[list[TrajectoryPathDTO]]` — static or class method.
- **D-10:** Grouper raises `ValueError` for empty `path_id`, duplicate `path_id`, or invalid `motion_type`. Negative `blend_radius` normalized to `0.0` before grouping (in DTO, not grouper).
- **D-11:** `TrajectoryGrouper.group()` called at top of `execute_callback`.
- **D-12:** `goal_callback` rejection order: (1) lifecycle state NOT `PRIMARY_STATE_ACTIVE`; (2) `_is_executing` is True.
- **D-13:** `_is_executing` set at entry of `execute_callback`, cleared on exit. Protected by `threading.Lock`.
- **D-14:** Lifecycle state check uses integer ID comparison (not label string).
- **D-15:** Four types: `MotionTypeEnum`, `FeedbackStatusEnum`, `TrajectoryPathDTO` (frozen), `TrajectoryGoalDTO` (frozen).
- **D-16:** Five test targets in `tests/unit/`.

### the agent's Discretion
- None specified — all implementation details were locked in CONTEXT.md.

### Deferred Ideas (OUT OF SCOPE)
- Cross-path validation (blend radius logic, cartesian_speed > 0, CIRC validity) — Phase 3.
- Action server cancellation handling — Phase 3/4.
- `TrajectoryGoalDTO.get_execution_groups()` convenience method — not selected; plain grouper utility chosen instead.
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| ACT-01 | `ExecuteTrajectory` action defined in `action/ExecuteTrajectory.action` with goal containing a list of trajectory paths | Phase 1 complete — interface verified importable |
| ACT-02 | Each `TrajectoryPath` specifies UUID4 path ID, motion type, target pose, blend radius, cartesian speed, acceleration, CIRC fields | All fields verified in `TrajectoryPath.msg` source |
| ACT-03 | Feedback publishes `{status: executing\|completed, trajectory_path_id: list[string]}` per blended group | Feedback type verified in `ExecuteTrajectory.action`; stub pattern described |
| ACT-04 | Result returns `{success: bool, error_message: string, trajectory_paths_completed: list[string]}` | Result type verified in `ExecuteTrajectory.action`; `goal_handle.succeed(result)` API confirmed |
| ACT-05 | Trajectory execution node is a `rclpy.lifecycle.LifecycleNode` with correct lifecycle transitions | All lifecycle APIs verified; critical pitfall about `get_current_state()` documented |
</phase_requirements>

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Lifecycle state machine | ROS2 node (`rclpy.lifecycle.LifecycleNode`) | — | LifecycleNode provides the state machine; callbacks wired by subclass |
| Action server goal handling | ROS2 node (`rclpy.action.ActionServer`) | — | `goal_callback` / `execute_callback` run inside the node's executor |
| Goal validation | Node — `goal_callback` | DTO layer (Pydantic) | Structural check at ROS2 boundary; Pydantic enforces field-level invariants |
| Blend grouping logic | Utility class (`TrajectoryGrouper`) | — | Pure function; no ROS2 or hardware dependency; independently testable |
| Data model conversion | DTO layer (`TrajectoryPathDTO`, `TrajectoryGoalDTO`) | — | Converts ROS2 message fields to typed Python objects at the node boundary |
| Concurrency guard | Node — `threading.Lock` | — | `_is_executing` flag shared between `goal_callback` (read) and `execute_callback` (write) |

---

## Standard Stack

### Core (all verified in devcontainer)

| Library | Version | Purpose | Status |
|---------|---------|---------|--------|
| `rclpy.lifecycle.LifecycleNode` | ROS2 Jazzy | Base class for the node | [VERIFIED: installed rclpy] |
| `rclpy.action.ActionServer` | ROS2 Jazzy | Action server implementation | [VERIFIED: installed rclpy] |
| `lifecycle_msgs.msg.State` | ROS2 Jazzy | Lifecycle state constants | [VERIFIED: `PRIMARY_STATE_ACTIVE = 3`] |
| `pydantic.BaseModel` | 2.13.4 | DTO data models | [VERIFIED: `pip show pydantic`] |
| `pydantic.field_validator` | 2.13.4 | Field-level validation and coercion | [VERIFIED: import tested] |
| `threading.Lock` | stdlib | `_is_executing` concurrency guard | [VERIFIED: stdlib] |
| `rcl_interfaces.msg.ParameterDescriptor` | ROS2 Jazzy | Parameter declaration descriptions | [VERIFIED: `ParameterDescriptor(description=...)` works] |

### No new packages required
All dependencies for Phase 2 are already declared in `package.xml` (`rclpy`, `python3-pydantic`, `lifecycle_msgs`) and installed in the devcontainer. **No `pip install` or `package.xml` changes needed.**

---

## Package Legitimacy Audit

No new external packages are being installed in Phase 2. All libraries used are either ROS2 built-ins or already-declared dependencies from Phase 1.

---

## Architecture Patterns

### Recommended Project Structure (new files in Phase 2)

```
src/movement_controller/
├── movement_controller/
│   ├── ur_movement_controller.py        ← NEW (Phase 2 primary deliverable)
│   ├── enums/
│   │   ├── __init__.py                  ← existing stub
│   │   ├── motion_type_enum.py          ← NEW
│   │   └── feedback_status_enum.py      ← NEW
│   ├── models/
│   │   ├── __init__.py                  ← existing stub
│   │   ├── trajectory_path_dto.py       ← NEW
│   │   └── trajectory_goal_dto.py       ← NEW
│   └── utils/
│       ├── __init__.py                  ← existing stub
│       └── trajectory_grouper.py        ← NEW
└── tests/
    └── unit/
        ├── test_imports.py              ← existing (Phase 1)
        ├── test_enums_and_dtos.py       ← NEW
        ├── test_trajectory_grouper.py   ← NEW
        └── test_ur_movement_controller.py ← NEW
```

**CMakeLists.txt must be updated** to add `ament_add_pytest_test` entries for each new test file (see Pitfall 4 below).

**setup.py must be updated** to add the node entry point: `'ur_movement_controller = movement_controller.ur_movement_controller:main'`.

---

### Pattern 1: LifecycleNode Subclass

**What:** Override lifecycle callbacks and return `TransitionCallbackReturn.SUCCESS`.
**Verification:** Confirmed via `inspect.getsource(LifecycleNodeMixin)` — callbacks receive `LifecycleState` NamedTuple with `.label` and `.state_id` fields.

```python
# Source: verified from rclpy.lifecycle.node installed in devcontainer
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

class URMovementController(LifecycleNode):

    def __init__(self, node_name: str = 'ur_movement_controller') -> None:
        super().__init__(node_name)
        self._action_server = None
        self._is_executing: bool = False
        self._executing_lock = threading.Lock()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        self.declare_parameter(
            'action_server_name',
            'movement_controller/execute_trajectory',
            ParameterDescriptor(description='ROS2 action server name')
        )
        self.declare_parameter(
            'moveit_group_name',
            'ur_manipulator',
            ParameterDescriptor(description='MoveIt2 planning group name')
        )
        action_server_name = self.get_parameter('action_server_name').value
        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            action_server_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        return TransitionCallbackReturn.SUCCESS
```

---

### Pattern 2: Lifecycle State Check in goal_callback (CRITICAL CORRECTION)

**⚠️ CRITICAL FINDING:** `self.get_current_state()` **does NOT exist** in rclpy Jazzy.  
`grep -r "get_current_state" /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/` returns **zero results**.

The CONTEXT.md (D-14) states to use `self.get_current_state().id` — this is **incorrect** and will raise `AttributeError` at runtime.

**Correct API** (verified from `rclpy.lifecycle.node` internals):

```python
# _state_machine.current_state returns a tuple (state_id: int, label: str)
# [0] = integer state ID, [1] = label string
# This is confirmed by the internal __on_get_state handler:
#   resp.current_state.id, resp.current_state.label = self._state_machine.current_state

from lifecycle_msgs.msg import State

def _goal_callback(self, goal: ExecuteTrajectory.Goal) -> GoalResponse:
    # Check 1: lifecycle state
    if self._state_machine.current_state[0] != State.PRIMARY_STATE_ACTIVE:
        self.get_logger().error(
            'Goal rejected: node not in ACTIVE state '
            f'(current: {self._state_machine.current_state[1]})'
        )
        return GoalResponse.REJECT
    # Check 2: concurrent execution guard
    with self._executing_lock:
        if self._is_executing:
            self.get_logger().error('Goal rejected: another goal is already executing')
            return GoalResponse.REJECT
    # Check 3: structural validation (D-03)
    if not goal.paths:
        self.get_logger().error('Goal rejected: paths list is empty')
        return GoalResponse.REJECT
    for path in goal.paths:
        if not path.path_id:
            self.get_logger().error('Goal rejected: path_id is empty')
            return GoalResponse.REJECT
        if path.motion_type not in ('LIN', 'PTP', 'CIRC'):
            self.get_logger().error(f'Goal rejected: invalid motion_type {path.motion_type!r}')
            return GoalResponse.REJECT
    return GoalResponse.ACCEPT
```

**State constants** (verified: `State.PRIMARY_STATE_ACTIVE = 3`):
| Constant | Value |
|----------|-------|
| `State.PRIMARY_STATE_UNKNOWN` | 0 |
| `State.PRIMARY_STATE_UNCONFIGURED` | 1 |
| `State.PRIMARY_STATE_INACTIVE` | 2 |
| `State.PRIMARY_STATE_ACTIVE` | 3 |
| `State.PRIMARY_STATE_FINALIZED` | 4 |

---

### Pattern 3: execute_callback — Stub Feedback & Result

**What:** Async callback — receives `ServerGoalHandle`. Sets `_is_executing`, calls `TrajectoryGrouper.group()`, sends per-group feedback pairs, returns result.

```python
# Source: verified from rclpy.action.server installed in devcontainer
# execute_callback can be sync or async (uses await_or_execute internally)
async def _execute_callback(self, goal_handle: ServerGoalHandle) -> ExecuteTrajectory.Result:
    with self._executing_lock:
        self._is_executing = True
    try:
        # Convert ROS2 message to Pydantic DTOs
        paths = [TrajectoryPathDTO.from_ros_msg(p) for p in goal_handle.request.paths]
        groups = TrajectoryGrouper.group(paths)

        for group in groups:
            path_ids = [p.path_id for p in group]
            # Feedback: executing
            fb = ExecuteTrajectory.Feedback()
            fb.status = FeedbackStatusEnum.EXECUTING.value
            fb.trajectory_path_ids = path_ids
            goal_handle.publish_feedback(fb)
            # Feedback: completed
            fb2 = ExecuteTrajectory.Feedback()
            fb2.status = FeedbackStatusEnum.COMPLETED.value
            fb2.trajectory_path_ids = path_ids
            goal_handle.publish_feedback(fb2)

        result = ExecuteTrajectory.Result()
        result.success = True
        result.error_message = ''
        result.trajectory_paths_completed = [p.path_id for p in paths]
        goal_handle.succeed()
        return result
    except Exception as e:
        self.get_logger().error(f'Execution failed: {e}')
        result = ExecuteTrajectory.Result()
        result.success = False
        result.error_message = str(e)
        goal_handle.abort()
        return result
    finally:
        with self._executing_lock:
            self._is_executing = False
```

**Key API notes** (verified):
- `goal_handle.request` → the `ExecuteTrajectory.Goal` message
- `goal_handle.publish_feedback(feedback_msg)` → sends feedback
- `goal_handle.succeed()` → marks goal succeeded (takes no required args); result is returned from the callback
- `goal_handle.abort()` → marks goal aborted; result is returned from the callback

---

### Pattern 4: Pydantic v2 DTOs — TrajectoryPathDTO

```python
# Pydantic 2.13.4 verified
from pydantic import BaseModel, field_validator
from movement_controller.enums.motion_type_enum import MotionTypeEnum

class TrajectoryPathDTO(BaseModel, frozen=True):
    """DTO mirroring TrajectoryPath.msg fields."""
    path_id: str = Field(description='UUID4 path identifier, non-empty')
    motion_type: MotionTypeEnum = Field(description='LIN, PTP, or CIRC')
    blend_radius: float = Field(default=0.0, description='Blend radius in metres (negative → 0.0)')
    cartesian_speed: float = Field(default=0.0, description='End-effector speed m/s')
    acceleration: float = Field(default=0.0, description='End-effector acceleration m/s²')
    tool_frame: str = Field(default='', description='Tool frame override; empty → tool0')
    circ_type: str = Field(default='', description='CIRC point interpretation: interim or center')
    # target_pose and circ_point stored as raw ROS msg objects (or as serialized dicts)

    @field_validator('path_id')
    @classmethod
    def validate_path_id_non_empty(cls, v: str) -> str:
        if not v:
            raise ValueError('path_id must be non-empty')
        return v

    @field_validator('blend_radius', mode='before')
    @classmethod
    def normalize_negative_blend_radius(cls, v: float) -> float:
        return 0.0 if v < 0 else float(v)
```

**Pydantic v2 field_validator syntax** (verified):
- Use `mode='before'` for coercion/normalization before type validation
- Decorate with `@classmethod` (required in Pydantic v2)
- Validator raises `ValueError` for invalid values

---

### Pattern 5: Enum Classes

```python
from enum import Enum

class MotionTypeEnum(str, Enum):
    LIN = "LIN"
    PTP = "PTP"
    CIRC = "CIRC"

class FeedbackStatusEnum(str, Enum):
    EXECUTING = "executing"
    COMPLETED = "completed"
```

**Note:** `str, Enum` inheritance makes Pydantic v2 serialize as plain strings (not `"MotionTypeEnum.LIN"`), which matches the ROS2 string field values.

---

### Pattern 6: TrajectoryGrouper

```python
class TrajectoryGrouper:
    """Groups trajectory paths into blended execution groups."""

    @staticmethod
    def group(paths: list[TrajectoryPathDTO]) -> list[list[TrajectoryPathDTO]]:
        if not paths:
            raise ValueError('paths list must not be empty')
        
        # Pre-validate
        seen_ids: set[str] = set()
        for path in paths:
            if not path.path_id:
                raise ValueError('path_id must be non-empty')
            if path.path_id in seen_ids:
                raise ValueError(f'Duplicate path_id: {path.path_id!r}')
            seen_ids.add(path.path_id)
            # motion_type already validated by MotionTypeEnum in DTO
        
        groups: list[list[TrajectoryPathDTO]] = []
        for i, path in enumerate(paths):
            if i == 0 or path.blend_radius <= 0:
                groups.append([path])
            else:
                groups[-1].append(path)
        return groups
```

**Grouper acceptance test** (from D-07 example):
- Input: `[t0(br=0.5), t1(br=0), t2(br=0), t3(br=0.3), t4(br=0.3), t5(br=0.3), t6(br=0)]`
- Expected output: `[[t0], [t1], [t2, t3, t4, t5], [t6]]` → 4 groups → 8 feedback messages

---

### Pattern 7: ActionServer Construction in on_configure

**What:** ActionServer is created in `on_configure` (not `__init__`) and destroyed in `on_cleanup`.

**Why `on_configure` not `__init__`:** Parameters are not declared until `on_configure`; the action server name comes from a parameter. Also, creating the ActionServer requires the node to have completed its base class `__init__`.

**Why `ReentrantCallbackGroup`:** The `execute_callback` runs in a separate thread. Without `ReentrantCallbackGroup`, the callback group may block goal callbacks while `execute_callback` is running (depending on executor type).

```python
from rclpy.callback_groups import ReentrantCallbackGroup

self._action_server = ActionServer(
    self,
    ExecuteTrajectory,
    self.get_parameter('action_server_name').value,
    execute_callback=self._execute_callback,
    goal_callback=self._goal_callback,
    callback_group=ReentrantCallbackGroup(),
)
```

---

### Pattern 8: CMakeLists.txt — Test Registration

**What:** `ament_cmake_pytest` does NOT auto-discover tests. Every new test file must be explicitly registered with `ament_add_pytest_test` in `CMakeLists.txt`.

**When required:** Any time a new test file is added (Phase 2 adds 3 new files). Without this step, `colcon test` will NOT run the new tests.

```cmake
# Inside if(BUILD_TESTING) block — add AFTER the existing test_imports entry:
ament_add_pytest_test(test_enums_and_dtos
  "tests/unit/test_enums_and_dtos.py"
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
  ENV PYTEST_ADDOPTS=--import-mode=importlib
  TIMEOUT 60
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
ament_add_pytest_test(test_trajectory_grouper
  "tests/unit/test_trajectory_grouper.py"
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
  ENV PYTEST_ADDOPTS=--import-mode=importlib
  TIMEOUT 60
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
ament_add_pytest_test(test_ur_movement_controller
  "tests/unit/test_ur_movement_controller.py"
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
  ENV PYTEST_ADDOPTS=--import-mode=importlib
  TIMEOUT 60
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
```

**Rebuild required** after CMakeLists.txt changes (even with `--symlink-install`).

---

### Anti-Patterns to Avoid

- **`self.get_current_state()`**: Does not exist in rclpy Jazzy. Will raise `AttributeError`.
- **String label comparison** for lifecycle state (e.g., `== 'active'`): Brittle. Use integer constant `State.PRIMARY_STATE_ACTIVE`.
- **Creating ActionServer in `__init__`**: Node not fully initialized; parameter declarations haven't run yet.
- **Non-`@classmethod` Pydantic v2 validators**: Pydantic v2 requires `@classmethod` on `@field_validator`.
- **`time.sleep()` in stub execution**: Explicitly prohibited by D-06 (feedback sent immediately).
- **Returning non-`TransitionCallbackReturn` from lifecycle callbacks**: Will cause transition to the ERROR state. Return `TransitionCallbackReturn.SUCCESS` or `TransitionCallbackReturn.FAILURE`.
- **Forgetting `_is_executing = False` in finally block**: If `execute_callback` raises before the finally, the node will be permanently locked. Always use try/finally.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Lifecycle state machine | Custom state enum | `rclpy.lifecycle.LifecycleNode` | ROS2 lifecycle protocol; CLI tools (`ros2 lifecycle`) integrate with it |
| Action server protocol | Custom topic/service | `rclpy.action.ActionServer` | Handles goal IDs, cancellation, feedback, result; all built in |
| Field validation/coercion | Manual `if` checks | Pydantic `field_validator` | Automatic, composable, tested, frozen model enforcement |
| Enum validation | String membership check | `MotionTypeEnum(str, Enum)` with Pydantic | Pydantic validates against enum values automatically |

---

## Verified Interface File Contents

### `ExecuteTrajectory.action` Fields (Phase 1 artifact — verified)

```
# Goal
TrajectoryPath[] paths
---
# Result
bool success
string error_message
string[] trajectory_paths_completed
---
# Feedback
string status
string[] trajectory_path_ids
```

### `TrajectoryPath.msg` Fields (Phase 1 artifact — verified)

```
# Constants
string MOTION_TYPE_LIN="LIN"
string MOTION_TYPE_PTP="PTP"
string MOTION_TYPE_CIRC="CIRC"
string CIRC_TYPE_INTERIM="interim"
string CIRC_TYPE_CENTER="center"

# Fields
string path_id
string motion_type
geometry_msgs/PoseStamped target_pose
float64 blend_radius
float64 cartesian_speed
float64 acceleration
string tool_frame
string circ_type
geometry_msgs/Point circ_point
```

**TrajectoryPathDTO must mirror all fields except `target_pose` and `circ_point`** which are geometry_msgs types and should be stored as-is from the ROS2 message (no Pydantic conversion needed for Phase 2 since they're not inspected during stub execution). The planner should decide how to handle `target_pose`/`circ_point` in the DTO — either store as ROS2 msg objects (using `model_config = ConfigDict(arbitrary_types_allowed=True)`) or skip them for Phase 2 and only include the fields the grouper and feedback logic actually use.

---

## Common Pitfalls

### Pitfall 1: `get_current_state()` AttributeError
**What goes wrong:** Code uses `self.get_current_state().id` → `AttributeError: 'URMovementController' object has no attribute 'get_current_state'` the first time `goal_callback` is invoked.
**Why it happens:** The method does not exist in rclpy Jazzy (`grep` over the entire `rclpy` package returns zero results). It may exist in older ROS2 versions or in C++ bindings only.
**How to avoid:** Use `self._state_machine.current_state[0]` for the integer state ID.
**Warning signs:** `AttributeError` in `goal_callback` during first test.

### Pitfall 2: CMakeLists.txt - New Test Files Not Discovered
**What goes wrong:** `colcon test` reports all tests passing but new test files are never executed.
**Why it happens:** `ament_cmake_pytest` does NOT auto-discover tests — it requires explicit `ament_add_pytest_test(name "path/to/test.py" ...)` entries. Running `python -m pytest tests/unit/` directly works, but `colcon test` won't see new files.
**How to avoid:** Add `ament_add_pytest_test` entry for every new test file. Rebuild with `colcon build --symlink-install` after CMakeLists.txt change.
**Warning signs:** `colcon test-result --verbose` shows only `test_imports` test; new tests not listed.

### Pitfall 3: TransitionCallbackReturn Return Type
**What goes wrong:** Returning `True`, `None`, or an integer from a lifecycle callback → node enters error state and attempts the error recovery transition.
**Why it happens:** Default callback uses `TransitionCallbackReturn.SUCCESS`; any other truthy value breaks the state machine.
**How to avoid:** Always return `TransitionCallbackReturn.SUCCESS` (success case) or `TransitionCallbackReturn.FAILURE` (handled failure case).
**Warning signs:** Node transitions to `ERROR` state after `configure` or `activate`.

### Pitfall 4: ActionServer Created Before Parameters Declared
**What goes wrong:** ActionServer construction uses `action_server_name` parameter; if created in `__init__`, the parameter doesn't exist yet → `ParameterNotDeclaredException`.
**How to avoid:** Create `ActionServer` only inside `on_configure`, after `declare_parameter` calls.

### Pitfall 5: Pydantic v2 field_validator Missing @classmethod
**What goes wrong:** Pydantic v2 requires `@classmethod` on `@field_validator`; omitting it → `PydanticUserError` at model definition time.
**How to avoid:** Always pair `@field_validator(...) @classmethod` in that order.

### Pitfall 6: _is_executing Lock Scope in goal_callback
**What goes wrong:** Checking `_is_executing` inside `with self._executing_lock:` but then returning `GoalResponse.ACCEPT` — the lock is released, and between the check and the `execute_callback` setting `_is_executing = True`, a second goal arrives and is accepted.
**Why:** `goal_callback` and `execute_callback` run in different threads/callbacks. The lock in `goal_callback` should only guard the **read**; the actual `set` happens at the start of `execute_callback`. This race window is acceptable for Phase 2 (stub behavior). Document as a known Phase 2 limitation.

### Pitfall 7: Frozen Pydantic Models with `model_config arbitrary_types_allowed`
**What goes wrong:** `frozen=True` on `TrajectoryPathDTO` combined with `geometry_msgs/PoseStamped` field (which is a C extension type) requires `model_config = ConfigDict(arbitrary_types_allowed=True)` otherwise Pydantic raises a `PydanticSchemaGenerationError`.
**How to avoid:** Add `model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)` if storing ROS2 message objects in the DTO. Consider storing only the fields needed (skip `target_pose` and `circ_point` for Phase 2 stub).

---

## Runtime State Inventory

> Omitted — greenfield Python implementation; no rename/migration involved.

---

## Environment Availability

| Dependency | Required By | Available | Notes |
|------------|------------|-----------|-------|
| `rclpy` (Jazzy) | LifecycleNode, ActionServer | ✓ | `/opt/ros/jazzy/` |
| `lifecycle_msgs` | `State.PRIMARY_STATE_ACTIVE` | ✓ | Jazzy base install |
| `pydantic` 2.x | DTOs | ✓ | 2.13.4 confirmed |
| `pytest` | Unit tests | ✓ | In devcontainer |
| `ament_cmake_pytest` | `colcon test` | ✓ | In devcontainer |
| Python `threading` | `_is_executing` lock | ✓ | stdlib |

**No missing dependencies.** All Phase 2 needs are satisfied by Phase 1's `package.xml` and the devcontainer.

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest + ament_pytest |
| Config file | `src/movement_controller/setup.cfg` — `[tool:pytest]` section already present |
| Quick run (direct) | `python -m pytest src/movement_controller/tests/unit/ -v` (from workspace root, after `source install/setup.bash`) |
| Full suite command | `colcon test --packages-select movement_controller && colcon test-result --verbose` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Test File | Automated Command |
|--------|----------|-----------|-----------|-------------------|
| ACT-01/02 | DTO field validation: motion_type, path_id, blend_radius coercion | unit | `test_enums_and_dtos.py` | `pytest tests/unit/test_enums_and_dtos.py -v` |
| ACT-03 | Feedback sequence: executing→completed pairs per group | unit | `test_ur_movement_controller.py` | `pytest tests/unit/test_ur_movement_controller.py -v` |
| ACT-04 | Result: trajectory_paths_completed echoes all path IDs | unit | `test_ur_movement_controller.py` | `pytest tests/unit/test_ur_movement_controller.py -v` |
| ACT-05 | Lifecycle transitions accepted/rejected correctly | unit | `test_ur_movement_controller.py` | `pytest tests/unit/test_ur_movement_controller.py -v` |
| D-07 | Grouper: correct group formation with blend algorithm | unit | `test_trajectory_grouper.py` | `pytest tests/unit/test_trajectory_grouper.py -v` |

### Wave 0 Gaps

- [ ] `tests/unit/test_enums_and_dtos.py` — covers `TrajectoryPathDTO` validation, `TrajectoryGoalDTO`, enum values
- [ ] `tests/unit/test_trajectory_grouper.py` — covers `TrajectoryGrouper.group()` with the 7-path D-07 acceptance scenario plus edge cases
- [ ] `tests/unit/test_ur_movement_controller.py` — covers lifecycle transitions, goal rejection conditions, stub feedback sequence
- [ ] `CMakeLists.txt` updates — `ament_add_pytest_test` entries for all 3 new test files (triggers colcon test rebuild)

### Sampling Rate
- **Per task commit:** `python -m pytest src/movement_controller/tests/unit/ -v`
- **Per wave merge:** `colcon build --symlink-install && colcon test --packages-select movement_controller && colcon test-result --verbose`
- **Phase gate:** Full suite green before marking Phase 2 complete

---

## Security Domain

Phase 2 is a pure ROS2 intra-process communication layer with no external inputs beyond ROS2 action goals. Applicable ASVS categories:

| ASVS Category | Applies | Control |
|---------------|---------|---------|
| V5 Input Validation | Yes (goal messages) | Pydantic v2 `field_validator` + `goal_callback` structural checks |
| V2 Authentication | No | ROS2 localhost-only in Phase 2; no auth layer |
| V4 Access Control | No | Single-node, no multi-tenant |
| V6 Cryptography | No | No secrets or data at rest |

**Threat relevant to Phase 2:**
- **Malformed goal injection**: A ROS2 client sends a `TrajectoryPath` with `motion_type = "INVALID"` or negative `blend_radius`. Mitigated by `goal_callback` validation (returns `GoalResponse.REJECT`) and Pydantic coercion in DTOs.

---

## Open Questions

1. **TrajectoryPathDTO — how to handle `target_pose` and `circ_point` fields?**
   - What we know: These are `geometry_msgs/PoseStamped` and `geometry_msgs/Point` C extension types. Pydantic can't validate them without `arbitrary_types_allowed=True`.
   - What's unclear: Should Phase 2 DTO omit them (since stub execution doesn't use them) or include them (establishes the complete DTO for Phase 3)?
   - Recommendation: Include them with `model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)` to establish the complete interface now. Phase 3 will use these fields directly.

2. **`main()` entrypoint for node**
   - What we know: `setup.py` has an empty `console_scripts` section.
   - What's unclear: Whether Phase 2 should add the entrypoint (no launch file yet, can't test it end-to-end).
   - Recommendation: Add the `console_scripts` entry in `setup.py` so Phase 7 launch files can reference it without changes.

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `ReentrantCallbackGroup` is the correct group for `ActionServer` when concurrent goal rejection (not concurrent execution) is desired | Pattern 7 | Could cause deadlock or missed goal rejections with wrong group type |
| A2 | `goal_handle.succeed()` returns the result from the callback return value (not as an argument) | Pattern 3 | If `succeed(result)` is the correct calling convention, stub would send wrong result |

> **Note on A2**: Inspected `ServerGoalHandle.succeed(self, response=None)` signature. The result IS returned from the callback directly. Calling `goal_handle.succeed()` (no arg) then `return result` is the correct pattern. The `response` parameter in `succeed()` is not used in the standard `execute_callback` path.

---

## Sources

### Primary (HIGH confidence)
- Verified via `inspect.getsource()` against installed `/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/` in devcontainer
- Verified via `grep -r` against installed rclpy package
- All code patterns were validated against the actual installed rclpy Jazzy version

### Secondary (CITED)
- `.github/copilot-instructions.md` — project conventions (LifecycleNode pattern, async+callback, BSD-3-Clause, ParameterDescriptor)
- Phase 1 CONTEXT.md — interface field decisions (D-01 through D-16)
- Phase 1 generated interface files — direct source read

### Tertiary (ASSUMED)
- See Assumptions Log above.

---

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — all verified against installed packages
- Architecture: HIGH — patterns derived from actual rclpy source inspection
- Pitfalls: HIGH — Pitfall 1 confirmed by failed grep; others from direct API inspection
- Interface fields: HIGH — read directly from source files

**Research date:** 2026-05-27
**Valid until:** 2027-05-27 (rclpy Jazzy is LTS; APIs are stable)
