# STATE.md — movement_controller

> Project memory. Updated at phase transitions and milestone boundaries.

## Current Status

**Phase:** 1 — Package Scaffold & Interface Definitions  
**Current Phase:** Planned  
**Next Action:** Run `/gsd-execute-phase 1` to execute Phase 1

## Phase History

| Phase | Status | Completed |
|-------|--------|-----------|
| 1 — Package Scaffold & Interface Definitions | Planned | — |
| 2 — LifecycleNode & Action Server Skeleton | Not started | — |
| 3 — MoveIt2 + PILZ Single-Path Execution | Not started | — |
| 4 — Look-Ahead Planning & Blended Multi-Path Execution | Not started | — |
| 5 — Motion Constraints | Not started | — |
| 6 — Scene Management Service | Not started | — |
| 7 — Launch Files & Simulation Validation | Not started | — |
| 8 — Real Hardware Validation | Not started | — |

## Open Decisions

| Decision | Status | Notes |
|----------|--------|-------|
| PILZ planner over OMPL | Committed | Deterministic LIN/PTP/CIRC profiles |
| MoveGroupSequence for blending | Committed | Native MoveIt2 blend mechanism |
| Look-ahead parallel planning | Committed | Background thread plans N+1 during N execution |
| Constraints as persistent node params | Committed | Not overridable per move |
| Abstract base now, UR10 only in v1 | Committed | `BaseMovementController` designed but only UR10 impl |

## Active Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| PILZ CIRC path requires valid intermediate point | High | Medium | Document and validate in Phase 3 tests |
| MoveGroupSequence blend radius constraints | Medium | Medium | Spike in Phase 4; fallback to sequential execution |
| sim-to-real joint limit differences | Medium | Medium | Test with conservative speeds in Phase 8 |
| `moveit_py` API gaps vs MoveIt Commander docs | High | Low | Consult moveit.picknik.ai directly; never guess API |

## Notes

- Initialized: 2026-05-26
- Devcontainer base: `ros:jazzy-ros-base` (Ubuntu 24.04)
- Target robot: Universal Robotics UR10 (classic, not e-Series)
- Simulation stack: Gazebo Harmonic + `fake_hardware_interface`
- All planning docs committed to git (`commit_docs: true`)

---
*Last updated: 2026-05-26 after initialization*
