---
phase: 01-package-scaffold-and-interface-definitions
plan: 04
subsystem: testing
tags: [pytest, colcon-test, smoke-test, gitignore, conftest, sys-path, importlib]

requires:
  - phase: 01-01
    provides: ament_add_pytest_test wired in CMakeLists.txt; setup.cfg for pytest config
  - phase: 01-02
    provides: Generated action/msg/srv Python bindings in install/
  - phase: 01-03
    provides: Importable movement_controller Python sub-packages

provides:
  - Passing colcon test baseline: 4/4 smoke tests green
  - test_imports.py verifying all 7 generated interfaces and 4 sub-packages are importable
  - conftest.py at package root preventing sys.path shadowing
  - .gitignore for ROS2 + Python artifacts
  - --import-mode=importlib in setup.cfg

affects: [all subsequent phases that add tests]

tech-stack:
  added: [pytest --import-mode=importlib, conftest.py sys.path cleanup]
  patterns:
    - Test directories without __init__.py — prevents pytest from adding package root to sys.path
    - conftest.py at rootdir cleans residual sys.path entries before test collection
    - --import-mode=importlib in setup.cfg + ENV PYTEST_ADDOPTS as belt-and-suspenders

key-files:
  created:
    - src/movement_controller/tests/unit/test_imports.py
    - src/movement_controller/conftest.py
    - src/movement_controller/.gitignore
  modified:
    - src/movement_controller/CMakeLists.txt
    - src/movement_controller/setup.cfg

key-decisions:
  - "Removed __init__.py from tests/ and tests/unit/ — the root cause of sys.path shadowing"
  - "Added --import-mode=importlib to setup.cfg and ENV PYTEST_ADDOPTS in CMakeLists.txt"
  - "conftest.py at rootdir as belt-and-suspenders sys.path cleanup"

patterns-established:
  - "Test directories have NO __init__.py — avoids pytest's package-based sys.path insertion"
  - "conftest.py at package root cleans sys.path before collection in all test invocation modes"
  - "ENV PYTEST_ADDOPTS in ament_add_pytest_test for per-test-suite flag injection"

requirements-completed: [PKG-05, PKG-06]

duration: 45min
completed: 2026-05-27
---

# Plan 01-04: CI Smoke Tests & Baseline Verification

**Established a green colcon test baseline with 4/4 smoke tests passing, resolving a pytest sys.path shadowing issue caused by __init__.py files in the test directories.**

## Performance

- **Duration:** ~45 min (includes debugging the sys.path shadowing issue)
- **Completed:** 2026-05-27
- **Tasks:** test_imports.py, conftest.py, .gitignore, CMakeLists.txt/setup.cfg fixes
- **Files modified:** 5 (created 2, modified 2, deleted 2)

## Accomplishments
- Created `tests/unit/test_imports.py` with 4 smoke tests verifying all 7 generated interfaces and 4 Python sub-packages are importable
- Resolved sys.path shadowing: pytest treated `tests/` as a Python package (had `__init__.py`), causing it to add the source `movement_controller/` directory to sys.path, which shadowed the installed merged package containing the generated interfaces
- Added conftest.py at package root that cleans residual sys.path entries and evicts stale sys.modules before collection
- Added `--import-mode=importlib` to both setup.cfg and CMakeLists.txt ENV PYTEST_ADDOPTS
- Created `.gitignore` covering build/, install/, log/, \_\_pycache\_\_/, \*.pyc
- Final result: `colcon test-result` → 5 tests, 0 errors, 0 failures

## Task Commits

1. **Initial test files + .gitignore** - `e19ac51` (feat(1-4): add smoke tests, .gitignore, and tests directory structure)
2. **Fix: remove __init__.py, add conftest.py, import-mode** - `b067200` (fix(1-4): remove tests/__init__.py to prevent sys.path shadowing)

## Files Created/Modified
- `src/movement_controller/tests/unit/test_imports.py` — 4 smoke tests for all interfaces and sub-packages
- `src/movement_controller/conftest.py` — sys.path cleanup hook at pytest rootdir
- `src/movement_controller/.gitignore` — Excludes build/, install/, log/, pycache artifacts
- `src/movement_controller/CMakeLists.txt` — Added ENV PYTEST_ADDOPTS=--import-mode=importlib; removed tests/__init__.py from files_modified
- `src/movement_controller/setup.cfg` — Added addopts = --import-mode=importlib
- **Deleted:** `src/movement_controller/tests/__init__.py` and `tests/unit/__init__.py`

## Decisions Made
- **Root cause**: `tests/__init__.py` presence caused pytest to treat the test directory as a Python package. In all import modes, pytest then adds the package root (where `tests/` lives) to sys.path. The package root is `src/movement_controller/`, which contains a `movement_controller/` source directory. This source directory shadowed the installed merged package that has the generated `action/`, `msg/`, `srv/` interfaces.
- **Fix**: Removed `__init__.py` from `tests/` and `tests/unit/`. Without `__init__.py`, pytest treats test files as standalone modules and does not add the parent directory to sys.path.
- **Belt-and-suspenders**: `conftest.py` at rootdir + `--import-mode=importlib` retained as defense-in-depth for any future test scenarios that might reintroduce the issue.

## Deviations from Plan

### Auto-fixed Issues

**1. [Test Infrastructure] pytest sys.path shadowing by source package**
- **Found during:** CI baseline verification
- **Issue:** `tests/__init__.py` caused pytest to add the package root to sys.path, where the source `movement_controller/` directory shadows the installed merged package. Resulted in `ModuleNotFoundError: No module named 'movement_controller.action'` for 3/4 smoke tests.
- **Fix:** Removed `tests/__init__.py` and `tests/unit/__init__.py`. Added conftest.py with sys.path cleanup. Added `--import-mode=importlib` to setup.cfg and CMakeLists.txt.
- **Files modified:** Deleted `tests/__init__.py`, `tests/unit/__init__.py`; created `conftest.py`; modified `CMakeLists.txt`, `setup.cfg`
- **Verification:** `colcon test-result` → 5 tests, 0 errors, 0 failures
- **Committed in:** `b067200`
