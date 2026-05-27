---
phase: "01"
phase_name: "Package Scaffold & Interface Definitions"
review_date: "2026-05-27"
depth: standard
files_reviewed: 1
files_reviewed_list:
  - src/movement_controller/conftest.py
findings:
  critical: 0
  warning: 3
  info: 2
  total: 5
status: "issues_found"
---

# Code Review — Phase 01: Package Scaffold & Interface Definitions

## Summary

Reviewed `src/movement_controller/conftest.py` at standard depth. The file implements a `pytest_configure` hook that manipulates `sys.path` to ensure the colcon-installed (generated) `movement_controller` package takes import priority over the bare source copy.

The core problem being solved is real: Python's default `''` (empty-string CWD) entry in `sys.path` causes `src/movement_controller/movement_controller/` (the bare source copy, lacking generated interfaces) to shadow the installed version when pytest is run from within the source directory. The fix — removing `''` from `sys.path` — is correct and produces the desired import outcome.

**The code works correctly** but carries a significant documentation error and two behavioral edge cases that could mislead future maintainers or cause subtle failures in non-standard invocation scenarios.

---

## Findings

### WR-001 — Docstring falsely attributes the problem to `--import-mode=prepend`; actual mode is `importlib`

**File:** [src/movement_controller/conftest.py](../../../src/movement_controller/conftest.py#L29)
**Lines:** 29–37 (module docstring), 52 (inline comment)

**Issue:**
The module docstring states:

> "When pytest's `--import-mode=prepend` adds this rootdir to `sys.path[0]`, it causes `movement_controller/` (the source package) to shadow the colcon-installed merged package."

But `setup.cfg` configures:
```ini
[tool:pytest]
addopts = --import-mode=importlib
```

Under `importlib` mode, pytest does **not** prepend the rootdir to `sys.path`. The actual problem is Python's built-in `''` (CWD) entry that is always present in `sys.path` at interpreter startup. When pytest is invoked from within `src/movement_controller/`, the CWD-relative lookup finds the bare source package before the installed one.

The fix (removing `''`) is correct and needed under importlib mode too. But the stated rationale is wrong **in a dangerous direction**: any maintainer who knows that importlib mode does not prepend the rootdir will read this docstring and conclude the conftest is redundant — then delete it. That deletion would silently reintroduce the shadowing bug and break all interface smoke tests (`test_imports.py`).

**Fix:**
Replace the module docstring to accurately describe the real mechanism:

```python
"""
Root conftest.py — ensure installed (generated) interfaces take priority over source.

This file MUST live at the pytest rootdir (same directory as setup.cfg) so that
pytest loads it before test collection begins.

Problem: Python's default sys.path always contains '' (the empty string), which
resolves to the current working directory at import time.  When pytest is invoked
from within the source root (e.g. `python -m pytest` or `colcon test`), CWD is
`src/movement_controller/`, so Python finds `movement_controller/` (the bare source
copy, lacking generated action/msg/srv interfaces) *before* the colcon-installed
merged package at `install/.../site-packages/`.  This hook removes the CWD-based
entries from sys.path so the installed package is found instead.

Note: This is independent of --import-mode.  Even under --import-mode=importlib
(the configured mode in setup.cfg), test source files are imported via importlib
without needing the rootdir on sys.path — but ordinary `import movement_controller`
statements inside those tests still use the standard import machinery and are
affected by sys.path.
"""
```

---

### WR-002 — `cwd` removal is over-broad: removes paths unrelated to the source package

**File:** [src/movement_controller/conftest.py](../../../src/movement_controller/conftest.py#L46)
**Lines:** 46, 50

**Issue:**
```python
cwd = os.getcwd()
# ...
if p not in ('', cwd, pkg_root)
```

`os.getcwd()` is captured at hook time. When `colcon test` runs, the working directory is the **build directory** (`build/movement_controller/`), not the source root. When a developer invokes pytest from the workspace root (`/workspaces/movement_controller`), `cwd` is the workspace root. In both cases `cwd != pkg_root`, and this condition removes an unrelated path from `sys.path`.

Those paths are typically not in `sys.path` anyway, so the removal is usually a no-op. However, if the workspace root or build directory is ever added to `sys.path` legitimately (by a colcon test hook, a workspace-level conftest, or a pytest plugin), this filter silently strips it without any diagnostic.

**Fix:**
Key the filter on `pkg_root` only. If removing `''` is also desired as a hardening step, document it explicitly rather than bundling it with the unrelated `cwd` variable:

```python
# Remove sys.path entries that resolve to this source root (prevents the bare
# source package from shadowing the installed one via the CWD '' entry).
sys.path[:] = [
    p for p in sys.path
    if p != ''
    and not (p and os.path.realpath(p) == os.path.realpath(pkg_root))
]
```

---

### WR-003 — Module eviction does not verify the evicted module originated from the source root

**File:** [src/movement_controller/conftest.py](../../../src/movement_controller/conftest.py#L55)
**Lines:** 55–59

**Issue:**
```python
stale = [
    k for k in sys.modules
    if k == 'movement_controller' or k.startswith('movement_controller.')
]
for key in stale:
    del sys.modules[key]
```

This unconditionally evicts **all** `movement_controller.*` modules regardless of where they were loaded from. If a pytest plugin or a conftest.py in a subdirectory has already imported `movement_controller` from the correct installed path (which is a plausible future scenario as the project grows), those modules are evicted unnecessarily. The subsequent re-import is correct, but any live Python references to attributes from the evicted modules (e.g., `from movement_controller.action import ExecuteTrajectory` at module level in a plugin) will point to the old object while new imports return the re-imported one, silently breaking `isinstance()` checks across the pre/post eviction boundary.

In the current codebase this is low-risk because `pytest_configure` fires before user code is imported. But the pattern is fragile for future growth.

**Fix:**
Limit eviction to modules whose `__file__` is provably inside the source root:

```python
stale = [
    k for k, mod in sys.modules.items()
    if (k == 'movement_controller' or k.startswith('movement_controller.'))
    and getattr(mod, '__file__', None)
    and os.path.realpath(getattr(mod, '__file__', ''))
       .startswith(os.path.realpath(pkg_root))
]
for key in stale:
    del sys.modules[key]
```

---

### IN-001 — `tests/integration/` directory is missing

**File:** [src/movement_controller/tests/](../../../src/movement_controller/tests/)

**Issue:**
Project conventions in `copilot-instructions.md` specify tests live in both `tests/unit/` and `tests/integration/`. Only `tests/unit/` exists. The root conftest.py is correctly scoped to cover both subtrees once integration tests are added. However, the missing directory means integration test files added in future phases will be placed without an established directory, increasing the chance of structural inconsistency.

**Fix:**
Create the directory as part of this scaffold phase:
```bash
mkdir -p src/movement_controller/tests/integration
touch src/movement_controller/tests/integration/.gitkeep
```

---

### IN-002 — No double-invocation guard on `pytest_configure`

**File:** [src/movement_controller/conftest.py](../../../src/movement_controller/conftest.py#L42)
**Line:** 42

**Issue:**
`pytest_configure` has no idempotency guard. The `sys.path` mutation is idempotent (the filter is safe to run twice), but the module eviction on a second invocation would re-evict modules that were freshly re-imported between calls. Certain pytest plugins can trigger `pytest_configure` multiple times in a session. This is unlikely to matter today but is a robustness gap.

**Fix:**
```python
_CONFIGURED = False

def pytest_configure(config):  # noqa: ARG001
    global _CONFIGURED
    if _CONFIGURED:
        return
    _CONFIGURED = True
    # ... rest of hook unchanged
```

---

_Reviewed: 2026-05-27T00:00:00Z_
_Reviewer: GitHub Copilot (gsd-code-reviewer)_
_Depth: standard_
