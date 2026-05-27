---
phase: 1
slug: package-scaffold-and-interface-definitions
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-27
---

# Phase 1 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest (via ament_cmake_pytest) |
| **Config file** | `setup.cfg` with `[tool:pytest]` section — Wave 0 installs |
| **Quick run command** | `python -m pytest tests/unit/test_imports.py -v` (requires sourced install) |
| **Full suite command** | `colcon test --packages-select movement_controller && colcon test-result --verbose` |
| **Estimated runtime** | ~30 seconds (build + test) |

---

## Sampling Rate

- **After every task commit:** Run `python -m pytest tests/unit/test_imports.py -v`
- **After every plan wave:** Run `colcon test --packages-select movement_controller && colcon test-result --verbose`
- **Before `/gsd-verify-work`:** Full suite must be green
- **Max feedback latency:** 60 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 1-01-01 | 01 | 1 | PKG-01 | — | N/A | build | `colcon build --symlink-install --packages-select movement_controller` | ❌ W0 | ⬜ pending |
| 1-01-02 | 01 | 1 | PKG-02 | — | N/A | grep | `grep -c ur_robot_driver package.xml` | ❌ W0 | ⬜ pending |
| 1-02-01 | 02 | 2 | PKG-03 | — | N/A | import-smoke | `colcon test --packages-select movement_controller` | ❌ W0 | ⬜ pending |
| 1-03-01 | 03 | 3 | PKG-04 | — | N/A | import-smoke | `colcon test --packages-select movement_controller` | ❌ W0 | ⬜ pending |
| 1-04-01 | 04 | 4 | PKG-05 | — | N/A | manual | — | N/A (manual) | ⬜ pending |
| 1-04-02 | 04 | 4 | PKG-06 | — | N/A | build | `colcon build --symlink-install --packages-select movement_controller` | ❌ W0 | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

- [ ] `tests/unit/test_imports.py` — stubs for PKG-03, PKG-04 (import smoke test)
- [ ] `tests/unit/__init__.py` — empty init for test package
- [ ] `tests/__init__.py` — empty init for tests root
- [ ] `setup.cfg` — pytest discovery config with `[tool:pytest] junit_family=xunit2`

*All test infrastructure is new — this is Phase 1.*

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| BSD-3-Clause header present on all source files | PKG-05 | Build tooling does not enforce headers; purely a code review concern | Review each file: CMakeLists.txt, package.xml, setup.py, setup.cfg, all .action/.srv/.msg files, all Python __init__.py files |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 60s
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
