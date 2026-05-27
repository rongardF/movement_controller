---
phase: 01-package-scaffold-and-interface-definitions
plan: 01
subsystem: infra
tags: [colcon, ament_cmake, rosidl, SKIP_INSTALL, python-package, ros2-jazzy]

requires: []
provides:
  - Hybrid ament_cmake + ament_cmake_python build system with SKIP_INSTALL workaround
  - rosidl interface generation wired for 7 interface files
  - Python package installed via ament_python_install_package + manual generated-interface overlay
  - setup.cfg with pytest configuration
  - package.xml with all runtime/build/test dependencies declared

affects: [02, 03, 04, all subsequent phases]

tech-stack:
  added: [ament_cmake, ament_cmake_python, rosidl_default_generators, Python3]
  patterns:
    - SKIP_INSTALL flag on rosidl_generate_interfaces prevents duplicate ament_python_install_package target
    - Manual install(DIRECTORY) overlays generated interfaces onto the installed Python package
    - find_package(Python3) used to resolve version-specific site-packages path

key-files:
  created:
    - src/movement_controller/CMakeLists.txt
    - src/movement_controller/package.xml
    - src/movement_controller/setup.py
    - src/movement_controller/setup.cfg
  modified: []

key-decisions:
  - "Used SKIP_INSTALL on rosidl_generate_interfaces to avoid duplicate ament_python_install_package registration (ros2/rosidl_python#141)"
  - "Added manual install(DIRECTORY) to overlay rosidl_generator_py output onto the installed Python package"
  - "Used find_package(Python3) to compute correct site-packages path at configure time"

patterns-established:
  - "SKIP_INSTALL pattern: rosidl_generate_interfaces(...SKIP_INSTALL) + manual install(DIRECTORY) for hybrid packages"
  - "Hybrid build: ament_cmake for interfaces + ament_cmake_python for Python source in one package.xml"

requirements-completed: [PKG-01, PKG-02, PKG-03, PKG-06, UR-01, UR-02]

duration: 8min
completed: 2026-05-27
---

# Plan 01-01: Hybrid Package Build System

**Established the ament_cmake + ament_cmake_python hybrid build with SKIP_INSTALL workaround, enabling both rosidl interface generation and Python package installation from a single package.**

## Performance

- **Duration:** ~8 min
- **Completed:** 2026-05-27
- **Tasks:** 4 (CMakeLists.txt, package.xml, setup.py, setup.cfg)
- **Files modified:** 4

## Accomplishments
- Created `CMakeLists.txt` with full SKIP_INSTALL workaround: `rosidl_generate_interfaces(...SKIP_INSTALL)` + `ament_python_install_package` + `install(DIRECTORY ...)` overlay
- Created `package.xml` with ament_cmake build type, all runtime/test deps including ur_robot_driver, ur_moveit_config, moveit_py, python3-pydantic
- Created minimal `setup.py` for non-colcon tool compatibility
- Created `setup.cfg` with `[tool:pytest]` section (junit_family=xunit2)

## Task Commits

1. **Wave 1: Hybrid build system (all 4 files)** - `7caf4c7` (feat(1-1): create hybrid package build system)

## Files Created/Modified
- `src/movement_controller/CMakeLists.txt` — Hybrid build config with SKIP_INSTALL, rosidl, ament_python_install_package, manual install overlay, ament_add_pytest_test
- `src/movement_controller/package.xml` — Package manifest with ament_cmake build type, full dep list
- `src/movement_controller/setup.py` — Minimal setup.py for non-colcon compatibility
- `src/movement_controller/setup.cfg` — pytest config (later updated with --import-mode=importlib in plan 01-04)

## Decisions Made
- **SKIP_INSTALL pattern** adopted as the canonical solution for ros2/rosidl_python#141 — the only stable way to combine `rosidl_generate_interfaces` and `ament_python_install_package` in a single Jazzy package without duplicate CMake target errors.
- **find_package(Python3)** used to dynamically resolve the Python version, making the `install(DIRECTORY)` destination version-agnostic.

## Deviations from Plan
None — plan executed exactly as written.
