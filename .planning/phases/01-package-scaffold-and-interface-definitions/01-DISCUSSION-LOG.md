# Phase 1: Discussion Log

**Phase:** 1 — Package Scaffold & Interface Definitions
**Date:** 2026-05-27
**Areas discussed:** 4 selected + 2 follow-up topics

---

## Area 1: CIRC Fields in TrajectoryPath.msg

**Question:** Flat vs sub-message for circ_type / circ_point
**Selected:** Flat — all fields directly in TrajectoryPath.msg; CIRC-specific fields ignored for LIN/PTP.

**Question:** circ_type as string or uint8 + constants
**Selected (freetext):** String field + string constants in the .msg file (e.g., `string CIRC_TYPE_INTERIM="interim"`). Keeps wire format as string; provides named constants in generated code.

**Question:** circ_point as Point or PointStamped
**Selected:** `geometry_msgs/Point` — no frame. Assumed same frame as target_pose header.

**Question:** Single target_pose sufficient?
**Selected:** Yes — single `geometry_msgs/PoseStamped target_pose`. No additional via-pose.

---

## Area 2: ManageScene.srv Operation Encoding

**Question:** Single multiplexed service vs multiple srv files
**Selected:** Multiple srv files — one per operation: `AddObject.srv`, `AttachObject.srv`, `DetachObject.srv`, `RemoveObject.srv`, `ModifyAcm.srv`.

**Question:** Merge add_primitive and add_mesh or separate?
**Selected:** Merge into AddObject.srv — single AddObject.srv with `shape_msgs/SolidPrimitive` for primitives and `string mesh_file_path` for meshes (discriminated by whether mesh_file_path is empty).

**Question:** Response type for scene services
**Selected:** `bool success` + `string error_message` + `string object_id` echo on all service responses.

**Question:** ModifyAcm request encoding
**Selected:** Pair-list: `string[] object_ids_a`, `string[] object_ids_b`, `bool allowed`.

**Follow-up (freetext):** User asked about attaching a `tool0_extension` object and using its tip as TCP for cartesian motion. Clarification given: MoveIt2 publishes TF frames for attached objects; PILZ's `tool_frame` can reference these frames. This surfaced the need for a `tool_frame` field in TrajectoryPath.msg (see Area: TCP Config below).

**Question (AttachObject):** Infer attach pose from scene vs explicit attach_pose field
**Selected:** Explicit `geometry_msgs/Pose attach_pose` in `AttachObject.srv` request — pose of object relative to link frame.

---

## Area 3: SceneObject.msg Shape Encoding

**Question:** shape_msgs/SolidPrimitive vs custom msg vs wrapper
**Selected:** Use `shape_msgs/SolidPrimitive` directly in `AddObject.srv` — avoids redefining what ROS2 already provides.

**Question:** Object ID strategy
**Selected (freetext):** Server-generated UUID4. ID returned in response, not provided by caller. Callers must store the UUID4 for subsequent operations.

**Question:** Pose field type
**Selected:** `geometry_msgs/PoseStamped` — includes frame_id for flexibility.

---

## Area 4: Python Stub Depth

**Question:** Empty __init__.py vs skeleton classes vs __all__ declarations
**Selected:** Truly empty stubs — only `__init__.py` with BSD-3-Clause header. No placeholder classes.

**Question:** Include ur_movement_controller.py stub?
**Selected:** No — Phase 2 concern only.

**Question:** Smoke test scope
**Selected:** Import interfaces + sub-packages — verify all generated interfaces importable from Python and all movement_controller sub-packages importable.

---

## Follow-up: TCP Configuration (emerged from ManageScene discussion)

**Question:** Per-path tool_frame in TrajectoryPath.msg vs node parameter vs always tool0
**Selected:** Per-path `string tool_frame` field in TrajectoryPath.msg. Empty string → defaults to `tool0` at planning time. Enables dynamic TCP targeting (e.g., tip of an attached extension object).

---

## Agent Discretion Items

- None — all decisions were made explicitly by the user.

---

## Deferred Ideas

- None from this discussion.
