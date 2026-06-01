# STATE.md — movement_controller

> Project memory. Updated at phase transitions and milestone boundaries.

## Current Status

**Phase:** 5 — Motion Constraints  
**Current Plan:** Complete (5/5 plans)  
**Next Action:** Begin Phase 6 — Scene Management Service

## Phase History

| Phase | Status | Completed |
|-------|--------|----------|
| 1 — Package Scaffold & Interface Definitions | Completed | 2026-05-27 |
| 2 — LifecycleNode & Action Server Skeleton | Completed | 2026-05-27 |
| 3 — MoveIt2 + PILZ Single-Path Execution | Completed | 2026-05-28 |
| 4 — Look-Ahead Planning & Blended Multi-Path Execution | Completed | 2026-05-29 |
| 5 — Motion Constraints | Completed | 2026-06-01 |
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
| ConstraintConfigDTO sentinel values | Committed | ±1e9 float sentinels (ROS2 float64 cannot represent Python inf) |
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
*Last updated: 2026-06-01 after Phase 5 completion — all 5 plans executed, 109 unit tests + 9 integration tests passing*
