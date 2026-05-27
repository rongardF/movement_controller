# Phase 1: Package Scaffold & Interface Definitions — Research

**Researched:** 2026-05-27
**Domain:** ROS2 Jazzy hybrid ament_cmake_python package scaffold; rosidl interface generation
**Confidence:** HIGH (core CMake/interface patterns verified against official docs and Jazzy source)

---

## Summary

Phase 1 establishes the `movement_controller` ROS2 package as a hybrid `ament_cmake` +
`ament_cmake_python` package that generates custom ROS2 interfaces (`.action`, `.msg`, `.srv`) AND
installs a Python module — all in a single package. This is non-trivial because of a
well-documented conflict between `rosidl_generate_interfaces` and `ament_python_install_package`
when used together in the same CMake project.

The core problem (VERIFIED: [ros2/rosidl_python#141](https://github.com/ros2/rosidl_python/issues/141)):
internally, `rosidl_generate_interfaces` calls `ament_python_install_package(${PROJECT_NAME} ...)`,
registering a CMake target `ament_cmake_python_symlink_movement_controller`. When yours is also
called, the CMake configuration fails with a duplicate target error. Additionally, the Jazzy
`ament_python_install_package.cmake` (verified from the `jazzy` branch source) explicitly throws
`FATAL_ERROR` when the same package name is installed twice via the
`AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES` list check.

**The fix**: pass `SKIP_INSTALL` to `rosidl_generate_interfaces`. This suppresses rosidl's internal
`ament_python_install_package` call. You then call `ament_python_install_package(${PROJECT_NAME})`
yourself (for source Python) and add a manual `install(DIRECTORY ...)` for the generated Python
interface files that rosidl would have installed.

**Primary recommendation:** struct all CMake in this exact order: `rosidl_generate_interfaces` with
`SKIP_INSTALL` → `ament_python_install_package` → manual install of generated Python interfaces →
`ament_export_dependencies` → `ament_package`.

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01:** CIRC-specific fields are **flat** in `TrajectoryPath.msg` — `circ_type` (string) and
  `circ_point` (geometry_msgs/Point). No sub-message extraction.
- **D-02:** `circ_type` uses string constants in the `.msg` file:
  `string CIRC_TYPE_INTERIM="interim"` / `string CIRC_TYPE_CENTER="center"`.
- **D-03:** `circ_point` is `geometry_msgs/Point` (no frame); its frame is assumed to match
  `target_pose` header.
- **D-04:** Each path has a single `geometry_msgs/PoseStamped target_pose`.
- **D-05:** Each `TrajectoryPath.msg` includes a `string tool_frame` field (empty = `tool0` at plan
  time).
- **D-06:** Scene management is 5 separate `.srv` files: `AddObject.srv`, `AttachObject.srv`,
  `DetachObject.srv`, `RemoveObject.srv`, `ModifyAcm.srv`.
- **D-07:** All scene service responses: `bool success`, `string error_message`, `string object_id`.
- **D-08:** AddObject object_id is **server-generated UUID4** — not provided in request.
- **D-09:** Geometry via `shape_msgs/SolidPrimitive primitive` + `string mesh_file_path`. When
  `mesh_file_path` non-empty → mesh mode; otherwise primitive mode.
- **D-10:** Object pose is `geometry_msgs/PoseStamped` (includes frame_id).
- **D-11:** AttachObject request: `string object_id`, `string link_name`, `geometry_msgs/Pose
  attach_pose` (relative to link frame — explicit, not inferred).
- **D-12:** Attaching does NOT auto-modify ACM; caller must call `ModifyAcm` separately.
- **D-13:** ModifyAcm uses pair-list: `string[] object_ids_a`, `string[] object_ids_b`, `bool
  allowed`.
- **D-14:** Sub-packages are **empty stubs** — only `__init__.py` files with BSD-3-Clause headers.
  No skeleton classes.
- **D-15:** `ur_movement_controller.py` is NOT created in Phase 1.
- **D-16:** Smoke test verifies: (1) all generated interface types importable; (2) all Python
  sub-packages importable.

### Agent's Discretion
No discretion areas specified.

### Deferred Ideas (OUT OF SCOPE)
No deferred ideas captured.
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| PKG-01 | ament_cmake_python hybrid layout (C++ for rosidl, Python for nodes) | §CMakeLists.txt Structure |
| PKG-02 | package.xml declares all runtime/test dependencies | §package.xml |
| PKG-03 | CMakeLists.txt generates all ROS2 interfaces via rosidl_generate_interfaces | §Interface Files, §CMakeLists.txt |
| PKG-04 | Python module layout: movement_controller/, models/, enums/, utils/, services/ | §Python Layout |
| PKG-05 | All source files carry BSD-3-Clause license header | §BSD-3-Clause Header |
| PKG-06 | `colcon build --symlink-install` succeeds with zero errors | §Common Pitfalls |
| UR-01 | Package depends on ur_robot_driver and ur_moveit_config | §package.xml |
| UR-02 | MoveIt2 planning group name = `ur_manipulator` | §SRDF Note |
</phase_requirements>

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Interface generation (.action/.srv/.msg → Python bindings) | CMake/rosidl | — | rosidl_generate_interfaces compiles IDL to language-specific code; pure build-time |
| Python node package installation | CMake/ament_cmake_python | — | ament_python_install_package manages Python egg/symlink install |
| Generated Python interface installation | CMake install() | ament_python_install_package | Due to the SKIP_INSTALL workaround, generated Python files need manual install |
| Python test discovery | CMake/ament_cmake_pytest | pytest | ament_cmake builds must enumerate tests; pytest executes them |
| BSD license header enforcement | Convention/human review | — | Runtime/build tooling does not check headers; purely a code review concern |

---

## Standard Stack

### Core (all VERIFIED via official ROS2 Jazzy docs and GitHub source)

| CMake Package | Version | Purpose | Why Standard |
|---------------|---------|---------|--------------|
| `ament_cmake` | bundled with Jazzy | Base CMake build type | Mandatory for ament_cmake packages |
| `ament_cmake_python` | bundled with Jazzy | Python install macros | Required for `ament_python_install_package` |
| `rosidl_default_generators` | bundled with Jazzy | IDL → C++/Python codegen | Standard interface generation pipeline |
| `rosidl_default_runtime` | bundled with Jazzy | Runtime typesupport | Required for importing generated interfaces |
| `ament_cmake_pytest` | bundled with Jazzy | pytest integration for CMake builds | Required to register pytest tests in ament_cmake packages |

### Interface Dependencies (VERIFIED)

| ROS2 Package | Used In | Declared as |
|--------------|---------|-------------|
| `geometry_msgs` | `TrajectoryPath.msg` (PoseStamped, Point), `AttachObject.srv` (Pose) | `<depend>` in package.xml; in rosidl DEPENDENCIES |
| `shape_msgs` | `AddObject.srv` (SolidPrimitive) | `<depend>` in package.xml; in rosidl DEPENDENCIES |
| `action_msgs` | Implicit dependency for action types | `<depend>` in package.xml; NOT in rosidl DEPENDENCIES (added by framework automatically) |

---

## Critical Finding: rosidl + ament_python_install_package Conflict

### What It Is

[VERIFIED: ros2/rosidl_python#141, ament/ament_cmake jazzy branch source]

In ROS2 Jazzy, calling `rosidl_generate_interfaces(${PROJECT_NAME} ...)` and
`ament_python_install_package(${PROJECT_NAME})` in the same `CMakeLists.txt` **always fails**
with one of two errors:

**Error 1 — Duplicate CMake target:**
```
CMake Error: add_custom_target cannot create target
"ament_cmake_python_symlink_movement_controller" because another target with
the same name already exists.
```

**Error 2 — Installed names check (added in Jazzy's ament_cmake):**
```
CMake Error: ament_python_install_package() a Python module file or package with
the same name 'movement_controller' has been installed before
```

The Jazzy branch of `ament_python_install_package.cmake` contains this explicit guard:
```cmake
if(package_name IN_LIST AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES)
  message(FATAL_ERROR
    "ament_python_install_package() a Python module file or package with "
    "the same name '${package_name}' has been installed before")
endif()
```

The `EXTEND_EXISTING` flag that would fix this (from PR ament/ament_cmake#587) is **NOT present
in Jazzy** — it is a rolling/post-Jazzy addition.

### The Workaround (SKIP_INSTALL)

Pass `SKIP_INSTALL` to `rosidl_generate_interfaces`. This prevents rosidl's internal
`ament_python_install_package` call. The generated Python interface files must then be
installed manually:

```cmake
# Step 1: Generate interfaces WITHOUT auto-installing generated Python
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  "msg/TrajectoryPath.msg"
  "srv/AddObject.srv"
  "srv/AttachObject.srv"
  "srv/DetachObject.srv"
  "srv/RemoveObject.srv"
  "srv/ModifyAcm.srv"
  DEPENDENCIES geometry_msgs shape_msgs
  SKIP_INSTALL
)

# Step 2: Install source Python package (no conflict now)
ament_python_install_package(${PROJECT_NAME})

# Step 3: Install the generated Python interface code manually
# rosidl generates Python files to ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/
# These must be co-located with source Python in the same Python namespace
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)
```

**Effect on `--symlink-install`:**
- `ament_python_install_package` **still** creates a symlink for the **source** Python files
  (`movement_controller/__init__.py`, models/, enums/, etc.) — so Python edits are live.
- The generated interface files (action/, msg/, srv/ subdirs) are **copied** (not symlinked)
  because they live in the build directory — but this is fine since changing interface files
  always requires a rebuild.

---

## CMakeLists.txt Structure (Exact)

[VERIFIED: official ROS2 Jazzy docs + ament_cmake_python How-To Guide]

```cmake
cmake_minimum_required(VERSION 3.8)
project(movement_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Required packages
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Python version — needed to compute the correct site-packages install path
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Message dependencies used in interface files
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# --------------------------------------------------------------------------
# ROS2 interface generation
# SKIP_INSTALL: prevents rosidl from calling ament_python_install_package
# internally, which would conflict with our own call below.
# See: https://github.com/ros2/rosidl_python/issues/141
# --------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  "msg/TrajectoryPath.msg"
  "srv/AddObject.srv"
  "srv/AttachObject.srv"
  "srv/DetachObject.srv"
  "srv/RemoveObject.srv"
  "srv/ModifyAcm.srv"
  DEPENDENCIES geometry_msgs shape_msgs
  SKIP_INSTALL
)

# --------------------------------------------------------------------------
# Python package installation
# --------------------------------------------------------------------------
# Install our Python source (symlinked with --symlink-install)
ament_python_install_package(${PROJECT_NAME})

# Install the generated Python interface code that SKIP_INSTALL skipped.
# rosidl_generator_py writes these files to ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/.
# Evaluated at install time (after build), so the directory will exist.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

# --------------------------------------------------------------------------
# Testing
# --------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # NOTE: ament_cmake_pytest does NOT auto-discover tests — list each file explicitly.
  ament_add_pytest_test(test_imports
    "tests/unit/test_imports.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

# --------------------------------------------------------------------------
# Finalize
# --------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

---

## package.xml (Complete)

[VERIFIED: official ROS2 Jazzy docs — Custom-ROS2-Interfaces tutorial + ament_cmake_python guide]

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>movement_controller</name>
  <version>0.1.0</version>
  <description>
    ROS2 package providing a vendor-agnostic action interface for executing
    collision-aware, blended multi-path trajectories on industrial robot arms
    using MoveIt2 and the PILZ motion planner.
  </description>
  <maintainer email="maintainer@example.com">Maintainer Name</maintainer>
  <license>BSD-3-Clause</license>

  <!-- Build tool -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <!-- Interface generation -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- Runtime interface support -->
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- Interface message dependencies -->
  <depend>geometry_msgs</depend>
  <depend>shape_msgs</depend>
  <depend>action_msgs</depend>

  <!-- Python runtime -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>python3-pydantic</exec_depend>

  <!-- MoveIt2 -->
  <exec_depend>moveit_py</exec_depend>
  <exec_depend>moveit_ros_planning_interface</exec_depend>

  <!-- UR dependencies (UR-01, UR-02) -->
  <exec_depend>ur_robot_driver</exec_depend>
  <exec_depend>ur_moveit_config</exec_depend>

  <!-- Marks this package as containing ROS2 interface definitions -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <!-- Test dependencies -->
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Notes:**
- `python3-pydantic` is the apt package name for Pydantic on Ubuntu 24.04 (if not installing via
  pip/venv). If managed via requirements.txt /`/opt/venv`, the exec_depend should be omitted or
  adjusted. [ASSUMED — verify exact package name for devcontainer setup]
- `moveit_py` is the MoveIt2 Python bindings package. Its exact `<exec_depend>` name follows the
  MoveIt2 packaging conventions for Jazzy. [ASSUMED — verify against `apt show ros-jazzy-moveit-py`]

---

## Interface Files (Exact Syntax)

[VERIFIED: official ROS2 Jazzy About-Interfaces docs + CONTEXT.md decisions D-01 through D-13]

### action/ExecuteTrajectory.action

```
# Goal: ordered list of trajectory paths to execute
TrajectoryPath[] paths
---
# Result
bool success
string error_message
string[] trajectory_paths_completed
---
# Feedback: emitted when blended execution starts and when it completes
string status
string[] trajectory_path_ids
```

**Notes:**
- `TrajectoryPath` is from the same package — no package prefix needed (same-package convention).
- `status` values at runtime: `"executing"` and `"completed"` (enforced by implementation, not IDL).
- `trajectory_path_ids` lists all path IDs included in the blended segment.

### msg/TrajectoryPath.msg

```
# String constants for motion type
string MOTION_TYPE_LIN="LIN"
string MOTION_TYPE_PTP="PTP"
string MOTION_TYPE_CIRC="CIRC"

# String constants for CIRC point interpretation (D-02)
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

**Note on string constants in ROS2 IDL (VERIFIED: About-Interfaces docs):**
- Constants are written as: `constanttype CONSTANTNAME=constantvalue`
- String constants: `string CONSTANT_NAME="value"` — single or double quotes both accepted
- Constant names **must be UPPERCASE** per ROS2 IDL specification
- Constants are accessible in Python as `TrajectoryPath.MOTION_TYPE_LIN`, etc.

### srv/AddObject.srv

```
# Request
shape_msgs/SolidPrimitive primitive
geometry_msgs/PoseStamped pose
string mesh_file_path
---
# Response
bool success
string error_message
string object_id
```

### srv/AttachObject.srv

```
# Request
string object_id
string link_name
geometry_msgs/Pose attach_pose
---
# Response
bool success
string error_message
string object_id
```

### srv/DetachObject.srv

```
# Request
string object_id
---
# Response
bool success
string error_message
string object_id
```

### srv/RemoveObject.srv

```
# Request
string object_id
---
# Response
bool success
string error_message
string object_id
```

### srv/ModifyAcm.srv

```
# Request — pair-list encoding: sets ACM entry for every (a_i, b_j) pair (D-13)
string[] object_ids_a
string[] object_ids_b
bool allowed
---
# Response
bool success
string error_message
string object_id
```

---

## Python Package Layout

[VERIFIED: copilot-instructions.md §Key Conventions + testing.md]

```
movement_controller/          ← ROS2 package root (colcon workspace: src/movement_controller/)
├── CMakeLists.txt
├── package.xml
├── setup.py                  ← minimal setup.py for non-colcon tool compatibility
├── setup.cfg                 ← pytest test discovery config
├── action/
│   └── ExecuteTrajectory.action
├── msg/
│   └── TrajectoryPath.msg
├── srv/
│   ├── AddObject.srv
│   ├── AttachObject.srv
│   ├── DetachObject.srv
│   ├── RemoveObject.srv
│   └── ModifyAcm.srv
├── tests/
│   └── unit/
│       └── test_imports.py   ← smoke test (D-16)
└── movement_controller/      ← Python package source (same name as ROS2 package)
    ├── __init__.py           ← BSD-3-Clause header + empty body
    ├── models/
    │   └── __init__.py       ← stub with BSD-3-Clause header
    ├── enums/
    │   └── __init__.py
    ├── utils/
    │   └── __init__.py
    └── services/
        └── __init__.py
```

**layout rules (from copilot-instructions.md):**
- Sub-packages are truly empty stubs in Phase 1 — only `__init__.py` with BSD header (D-14)
- `ur_movement_controller.py` NOT created in Phase 1 (D-15)
- `tests/` is a sibling of the Python package source, NOT nested inside it (testing.md)

---

## BSD-3-Clause License Header

[ASSUMED — standard BSD-3-Clause format; verify year/org against project ownership]

**Python files:**
```python
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
```

**CMake files:** Replace `#` comment prefix with `# ` (same format, same content).

**ROS2 interface files (`.action`, `.msg`, `.srv`):** Add comment block at top:
```
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
# ... (same text, using # comment style)
```

---

## Test Infrastructure

[VERIFIED: official ROS2 Jazzy ament_cmake_python How-To Guide + testing.md]

### Key Difference: ament_cmake_pytest vs ament_python

In `ament_cmake` packages (unlike pure `ament_python` packages), pytest test **auto-discovery
does NOT work**. Each test file MUST be explicitly registered in `CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_imports "tests/unit/test_imports.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()
```

### package.xml test dependencies

```xml
<test_depend>ament_cmake_pytest</test_depend>
<test_depend>pytest</test_depend>
```

Note: `ament_pytest` (the pure-Python package) is used for `ament_python` build-type packages.
For `ament_cmake` packages, `ament_cmake_pytest` is the correct dependency.

### setup.cfg (test discovery for direct pytest runs)

```ini
[tool:pytest]
junit_family = xunit2
```

### Smoke Test (test_imports.py)

```python
# Copyright (c) 2026, Movement Controller Contributors
# ... (BSD-3-Clause header)
"""Smoke test: verify all generated interfaces and Python sub-packages are importable."""


def test_action_execute_trajectory_importable():
    """Verify ExecuteTrajectory action type is importable after colcon build."""
    from movement_controller.action import ExecuteTrajectory  # noqa: F401
    assert hasattr(ExecuteTrajectory, 'Goal')
    assert hasattr(ExecuteTrajectory, 'Result')
    assert hasattr(ExecuteTrajectory, 'Feedback')


def test_msg_trajectory_path_importable():
    """Verify TrajectoryPath message type is importable."""
    from movement_controller.msg import TrajectoryPath  # noqa: F401


def test_srv_all_importable():
    """Verify all 5 scene management service types are importable."""
    from movement_controller.srv import AddObject  # noqa: F401
    from movement_controller.srv import AttachObject  # noqa: F401
    from movement_controller.srv import DetachObject  # noqa: F401
    from movement_controller.srv import RemoveObject  # noqa: F401
    from movement_controller.srv import ModifyAcm  # noqa: F401


def test_python_subpackages_importable():
    """Verify Python sub-package stubs are importable."""
    import movement_controller  # noqa: F401
    from movement_controller import models  # noqa: F401
    from movement_controller import enums  # noqa: F401
    from movement_controller import utils  # noqa: F401
    from movement_controller import services  # noqa: F401
```

### Running tests

```bash
# Full workflow (recommended — tests run in correct install environment)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select movement_controller
colcon test --packages-select movement_controller
colcon test-result --verbose

# Direct pytest (faster during development; requires sourced install)
source install/setup.bash
python -m pytest src/movement_controller/tests/ -v
```

---

## Architecture Patterns

### Recommended Project Structure
```
src/movement_controller/
├── CMakeLists.txt               # ament_cmake + rosidl + Python install
├── package.xml                  # build/exec/test dependencies
├── setup.py                     # minimal; for non-colcon Python tooling
├── setup.cfg                    # pytest config
├── action/
│   └── ExecuteTrajectory.action
├── msg/
│   └── TrajectoryPath.msg
├── srv/
│   ├── AddObject.srv
│   ├── AttachObject.srv
│   ├── DetachObject.srv
│   ├── RemoveObject.srv
│   └── ModifyAcm.srv
├── tests/
│   └── unit/
│       └── test_imports.py
└── movement_controller/
    ├── __init__.py
    ├── models/__init__.py
    ├── enums/__init__.py
    ├── utils/__init__.py
    └── services/__init__.py
```

### Pattern: Interface File Ordering in rosidl_generate_interfaces

The order of files passed to `rosidl_generate_interfaces` matters when types reference each other:
- `TrajectoryPath.msg` must be listed **before or alongside** `ExecuteTrajectory.action` (rosidl
  resolves same-package dependencies; order shouldn't matter in practice, but alphabetical within
  type groups is conventional).
- Same-package type references (`TrajectoryPath` in `ExecuteTrajectory.action`) use just the
  type name without package prefix.
- Cross-package type references use the full `package_name/TypeName` syntax.

### Pattern: Python Namespace Coexistence

After the full build, the Python namespace `movement_controller` contains both:
- **Source code** (symlinked): `models/`, `enums/`, `utils/`, `services/`, `__init__.py`
- **Generated interfaces** (copied): `action/`, `msg/`, `srv/`

Both live under `install/lib/python3.XX/site-packages/movement_controller/`. The two sets of files
occupy different subdirectories and do NOT conflict on disk — only at the CMake level (hence the
`SKIP_INSTALL` workaround).

### Anti-Patterns to Avoid

- **Never call `rosidl_generate_interfaces` without `SKIP_INSTALL` when also calling
  `ament_python_install_package` in Jazzy** — this is the primary build failure mode.
- **Never omit `<member_of_group>rosidl_interface_packages</member_of_group>` from package.xml**
  — rosidl needs this to correctly associate typesupport with the package.
- **Never put tests inside the Python package source directory** — `tests/` is a sibling of
  `movement_controller/`, not nested inside it.
- **Never use `ament_python` build type** for a package that generates rosidl interfaces — the
  build type must be `ament_cmake`.
- **Never reference packages in DEPENDENCIES that your `.msg`/`.srv`/`.action` files don't
  directly use** — only list packages for types you directly reference (geometry_msgs, shape_msgs).
  `action_msgs` is added automatically for action types.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Python test symlinks with colcon | Custom cmake install targets | `ament_python_install_package` | Handles egg-info, symlinks, compile-all correctly |
| Interface compilation | Python class stubs | `rosidl_generate_interfaces` | Rosidl generates typed Python classes with serialize/deserialize |
| Test timeout/env setup | Manual pytest env scripts | `ament_add_pytest_test` + `APPEND_ENV` | Correctly propagates ROS2 environment variables to test process |
| Package XML validation | Inline cmake checks | `ament_lint_auto` (`ament_xmllint`, `ament_pep8`) | If lint gates needed, use ament's tooling |

---

## Common Pitfalls

### Pitfall 1: rosidl + ament_python_install_package conflict (CRITICAL)

**What goes wrong:** CMake configuration fails with duplicate target or FATAL_ERROR about
package name already installed.

**Why it happens:** `rosidl_generate_interfaces` internally calls `ament_python_install_package`
for the generated Python bindings, creating a CMake target with the same name as the one you
create yourself. The Jazzy ament_cmake adds an explicit FATAL_ERROR guard for this.

**How to avoid:** Pass `SKIP_INSTALL` to `rosidl_generate_interfaces` and add a manual
`install(DIRECTORY ...)` for the generated Python files.

**Warning signs:** Error mentions `ament_cmake_python_symlink_movement_controller` target or
"Python module file or package with the same name has been installed before."

---

### Pitfall 2: Generated Python files not in PYTHONPATH during tests

**What goes wrong:** `colcon test` fails with `ImportError: cannot import name 'ExecuteTrajectory'`
even after successful build.

**Why it happens:** With `SKIP_INSTALL`, the generated Python files are in the build directory. The
`install(DIRECTORY ...)` copies them to the install space at install time, but `colcon test` sources
the install space — so this should work. HOWEVER, if install step was missed, or if testing without
the full install step, the imports will fail.

**How to avoid:** 
1. Always run `colcon build` before `colcon test` (not just build, but full install).
2. Use `APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py` in
   `ament_add_pytest_test` as a fallback that works even without the install step.

**Warning signs:** `ImportError` on interface imports but Python source imports succeed.

---

### Pitfall 3: Missing DEPENDENCIES in rosidl_generate_interfaces

**What goes wrong:** Build succeeds but runtime import of generated types raises cryptic
`ImportError` or the rosidl generation quietly produces incorrect types.

**Why it happens:** If `geometry_msgs` or `shape_msgs` are not in `DEPENDENCIES`, rosidl cannot
find the type definitions for referenced types (`geometry_msgs/PoseStamped`, etc.) during
code generation.

**How to avoid:** List ALL packages whose types are directly referenced in your `.msg`/`.srv`/
`.action` files. For this package: `DEPENDENCIES geometry_msgs shape_msgs`.

**Warning signs:** `colcon build` may succeed (rosidl might not error), but generated Python code
may have runtime issues or `colcon build` may fail with "undefined message type" during codegen.

---

### Pitfall 4: Forgetting `<member_of_group>rosidl_interface_packages</member_of_group>`

**What goes wrong:** Other packages that `<depend>movement_controller</depend>` cannot find the
generated interface types.

**Why it happens:** The `rosidl_interface_packages` group membership tag tells the ament/colcon
infrastructure that this package exports ROS2 interface types. Without it, downstream packages
won't correctly set up their typesupport dependencies.

**How to avoid:** Always include `<member_of_group>rosidl_interface_packages</member_of_group>` in
`package.xml` when defining ROS2 interfaces.

---

### Pitfall 5: Tests not discovered by colcon

**What goes wrong:** `colcon test` runs with no tests found; `colcon test-result` shows 0 tests.

**Why it happens:** Unlike `ament_python` packages, `ament_cmake_pytest` does NOT auto-discover
tests. Each test file must be explicitly registered with `ament_add_pytest_test`.

**How to avoid:** List every test file in `CMakeLists.txt` inside the `if(BUILD_TESTING)` block.
Add `<test_depend>ament_cmake_pytest</test_depend>` to `package.xml`.

---

### Pitfall 6: String constants must be UPPERCASE

**What goes wrong:** `colcon build` fails with rosidl error about constant naming.

**Why it happens:** ROS2 IDL requires constant names to be UPPERCASE (enforced by rosidl). Example:
`string motion_type_lin="LIN"` is **invalid**; `string MOTION_TYPE_LIN="LIN"` is **valid**.

**How to avoid:** All constant names in `.msg`/`.srv`/`.action` files must be UPPERCASE.

---

### Pitfall 7: `install(DIRECTORY ...)` evaluated before build

**What goes wrong:** The `install` call for the generated Python files tries to install from a
directory that doesn't exist at configure time, potentially causing cmake warnings.

**Why it happens:** CMake's `install(DIRECTORY ...)` is evaluated at install time (after build),
not at configure time — so missing directory at configure time is OK. However, some CMake
versions/configurations may warn or error if source doesn't exist at configure time.

**How to avoid:** CMake's `install(DIRECTORY ...)` with a non-existent source dir silently
installs nothing (no error in standard CMake). The directory WILL exist after build. The install
will succeed when invoked post-build. Verify by running `colcon build` fully before `source
install/setup.bash`.

---

## SRDF Note (UR-02)

The MoveIt2 planning group name is `ur_manipulator` — this name is defined in the `ur_moveit_config`
SRDF for UR robots [VERIFIED: UR Robot Driver docs + ur_moveit_config standard]. This is a
runtime/node-level concern and does NOT affect Phase 1 interface definitions. Documented here so
planners know the constant is `ur_manipulator` when writing smoke test assertions in later phases.

---

## Environment Availability

This phase has no external service dependencies beyond the devcontainer. Build tooling is
provided by the ROS2 Jazzy base image.

| Dependency | Required By | Available | Version | Notes |
|------------|-------------|-----------|---------|-------|
| `ros:jazzy-ros-base` image | All build steps | ✓ (devcontainer) | Jazzy / Ubuntu 24.04 | Base image; ament_cmake, rosidl, geometry_msgs, shape_msgs all bundled |
| Python 3.12 | ament_cmake_python, tests | ✓ | 3.12.x | Default in Ubuntu 24.04 |
| colcon | Build + test runner | ✓ | bundled | `python3-colcon-common-extensions` in base image |
| `ur_robot_driver` | package.xml exec_depend | May require apt install | Latest | Not in `ros:jazzy-ros-base` by default; needed for `colcon build` to resolve the dependency OR declared as `exec_depend` only (not build_depend), so build succeeds without it |
| `ur_moveit_config` | package.xml exec_depend | May require apt install | Latest | Same situation as ur_robot_driver |
| `moveit_py` | package.xml exec_depend | May require apt install | Latest | `ros-jazzy-moveit` or `ros-jazzy-moveit-py` |

**Note on UR and MoveIt2 dependencies**: These are `exec_depend` (runtime) in package.xml, not
`build_depend`. `colcon build` will succeed without them installed — they're only needed at
runtime. Phase 1 only builds the scaffold; nothing executes. HOWEVER, `colcon build` resolves
`exec_depend` for header/library linking. Since these are pure Python deps, the build should
succeed without them physically present (no linking required). Verify in the devcontainer's
Dockerfile that required apt packages are installed.

---

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | pytest (via ament_cmake_pytest) |
| Config file | `setup.cfg` with `[tool:pytest]` section |
| Quick run command | `python -m pytest tests/unit/test_imports.py -v` (requires sourced install) |
| Full suite command | `colcon test --packages-select movement_controller && colcon test-result --verbose` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| PKG-01 | ament_cmake_python layout builds | Build check | `colcon build --symlink-install --packages-select movement_controller` | ❌ Wave 0 |
| PKG-03 | Interface files compile | Import smoke test | `colcon test --packages-select movement_controller` | ❌ Wave 0 |
| PKG-04 | Python sub-packages importable | import smoke test | `colcon test --packages-select movement_controller` | ❌ Wave 0 |
| PKG-05 | BSD headers on all files | Manual/human review | — | N/A (manual) |
| PKG-06 | Zero build errors | Build check | `colcon build --packages-select movement_controller` | ❌ Wave 0 |
| UR-01 | ur_robot_driver/ur_moveit_config in package.xml | package.xml review | `grep -c ur_robot_driver package.xml` | ❌ Wave 0 |

### Wave 0 Gaps

- [ ] `tests/unit/test_imports.py` — covers PKG-03, PKG-04
- [ ] `tests/unit/__init__.py` — empty init to make tests directory a Python package
- [ ] `setup.cfg` — pytest discovery config

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `python3-pydantic` is the apt package name for Pydantic on Ubuntu 24.04 | package.xml | Build error—exec_depend may fail; fix: use `requirements.txt` + `/opt/venv` instead |
| A2 | `moveit_py` package exec_depend is `moveit_py` (apt: `ros-jazzy-moveit-py`) | package.xml | Build warning/error; fix: check apt package list in devcontainer |
| A3 | BSD-3-Clause header copyright holder and year: "Movement Controller Contributors, 2026" | §BSD Header | Legal/style issue; fix: update to correct org/year |
| A4 | `${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/` is the correct path for generated Python files | CMakeLists.txt | Manual install copies nothing; imports fail; fix: inspect build dir after first build |

---

## Open Questions

1. **Exact apt package name for Pydantic in devcontainer**
   - What we know: Pydantic v2 is required; `/opt/venv` is recommended for Python deps
   - What's unclear: Is `python3-pydantic` declared as `exec_depend` in package.xml, or managed entirely via `requirements.txt`?
   - Recommendation: Declare only ROS2-registered packages in `exec_depend`; Pydantic via requirements.txt → no exec_depend entry needed

2. **Exact apt package name for `moveit_py`**
   - What we know: `moveit_py` is the import name; `ros-jazzy-moveit-py` is likely the apt package
   - What's unclear: Exact `<exec_depend>` token for package.xml
   - Recommendation: Use `moveit_py` as the exec_depend token; colcon/rosdep will resolve

3. **Whether install(DIRECTORY ...) for generated Python files fires correctly in all build modes**
   - What we know: Standard CMake behavior evaluates `install()` at install-time (post-build)
   - What's unclear: Edge case behavior with `colcon build --symlink-install`
   - Recommendation: Verify once by running `ls install/lib/python*/site-packages/movement_controller/action/` after first build

---

## Sources

### Primary (HIGH confidence)
- [docs.ros.org/en/jazzy — ament_cmake_python How-To Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Python-Documentation.html)
  - Confirmed SKIP_INSTALL mention + rosidl conflict warning
  - Confirmed `ament_add_pytest_test` syntax for ament_cmake_pytest
- [docs.ros.org/en/jazzy — Creating custom msg and srv files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
  - Confirmed rosidl_generate_interfaces CMake syntax; DEPENDENCIES key; package.xml entries
- [docs.ros.org/en/jazzy — Implementing custom interfaces (single package)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
  - Confirmed single-package interface definition pattern; same-package reference syntax
- [docs.ros.org/en/jazzy — About Interfaces](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html)
  - Confirmed .action file format (goal --- result --- feedback); string constant UPPERCASE rule
- [github.com/ament/ament_cmake — jazzy branch — ament_python_install_package.cmake](https://github.com/ament/ament_cmake/blob/jazzy/ament_cmake_python/cmake/ament_python_install_package.cmake)
  - Confirmed the `AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES` FATAL_ERROR guard in Jazzy
  - Confirmed `ament_cmake_python_symlink_${package_name}` target name pattern
  - Confirmed NO `EXTEND_EXISTING` flag in Jazzy

### Secondary (MEDIUM confidence)
- [github.com/ros2/rosidl_python — issue #141](https://github.com/ros2/rosidl_python/issues/141)
  - Confirmed the conflict is well-known since 2021; no fix in Jazzy; `SKIP_INSTALL` workaround

### Tertiary (ASSUMED)
- Pydantic apt package name: assumed `python3-pydantic` — verify in devcontainer
- `moveit_py` exec_depend name: assumed `moveit_py` — verify with `apt show ros-jazzy-moveit-py`

---

## Metadata

**Confidence breakdown:**
- CMakeLists.txt structure: HIGH — verified against official Jazzy docs and Jazzy branch source
- Interface file syntax: HIGH — verified against official ROS2 About-Interfaces docs
- SKIP_INSTALL workaround: HIGH — verified against Jazzy ament_cmake source and GitHub issue
- package.xml structure: HIGH — verified against official docs
- BSD header format: ASSUMED — standard format, year/org need project-specific confirmation
- `moveit_py`/`pydantic` exec_depend names: ASSUMED — verify in devcontainer

**Research date:** 2026-05-27
**Valid until:** 2026-08-27 (stable ROS2 release — changes unlikely in Jazzy LTS)
