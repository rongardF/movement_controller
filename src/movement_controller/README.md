# movement_controller

ROS 2 Jazzy package that provides a vendor-agnostic action interface for executing
collision-aware multi-path trajectories on industrial robot arms using MoveIt 2
and the Pilz planner.

Current launch support is focused on Universal Robots (UR), but the package
structure and launch logic are prepared for adding additional robot families.

## Highlights

- Lifecycle-based controller node (`rclpy.lifecycle.LifecycleNode`)
- Action API for ordered trajectory execution:
	- Action: `movement_controller/execute_trajectory`
	- Type: `movement_controller/action/ExecuteTrajectory`
- Supports LIN, PTP, and CIRC path segments via `TrajectoryPath.msg`
- Collision-aware planning via MoveIt `plan_sequence_path`
- Optional path blending (grouped execution based on blend radius)
- Goal cancellation support (including MoveIt workaround topic publish)

## Package Layout

```text
src/movement_controller/
|- action/ExecuteTrajectory.action
|- msg/TrajectoryPath.msg
|- movement_controller/
|  |- movement_controller.py
|  |- models/
|  |- services/
|  |- enums/
|  `- utils/
|- launch/
|  |- movement_controller.launch.py
|  `- ur.launch.py
|- config/
|  |- joint_constraints.yaml
|  |- joint_limits.yaml
|  `- pilz_cartesian_limits.yaml
`- tests/
	 |- unit/
	 `- integration/
```

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2 for Jazzy
- UR ROS 2 driver (`ur_robot_driver`) for UR hardware setup
- Python 3.11+

If you are working in this repository's devcontainer, these are mostly already
provided. Use the project virtual environment and overlay setup as shown below.

## Build

To run RViz inside the devcontainer run the following command first to enable XHost forwarding.

```bash
sudo -i xhost +local:
```

From workspace root:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Launch full stack (UR driver + move_group + this node):

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch movement_controller movement_controller.launch.py \
	model:=ur10 \
	robot_ip:=192.168.1.9 \
	rviz:=true \
	debug:=false
```

Common launch arguments:

- `model` (default `ur10`)
- `robot_ip` (default `192.168.1.9`)
- `rviz` (`true` or `false`)
- `use_sim_time` (`true` or `false`)
- `debug` (`true` sets node log level to DEBUG)

### Lifecycle Activation

The controller is a lifecycle node. After launch, configure and activate it:

```bash
ros2 lifecycle set /movement_controller configure
ros2 lifecycle set /movement_controller activate
```

Check state:

```bash
ros2 lifecycle get /movement_controller
```

## Action Interface

### Goal

Action: `movement_controller/execute_trajectory`  
Type: `movement_controller/action/ExecuteTrajectory`

Goal field:

- `paths`: ordered list of `movement_controller/msg/TrajectoryPath`

### Result

- `success` (`bool`)
- `error_message` (`string`)
- `trajectory_paths_completed` (`string[]`)

### Feedback

- `status` (`string`)
- `trajectory_path_ids` (`string[]`)

Status values currently published by the node:

- `executing`
- `completed`

## TrajectoryPath Message

`TrajectoryPath.msg` contains fields used per path segment:

- `path_id` (UUID string expected)
- `motion_type` (`LIN`, `PTP`, `CIRC`)
- `target_pose` (`geometry_msgs/PoseStamped`)
- `blend_radius`
- `cartesian_speed`
- `cartesian_acceleration`
- `joint_speed`
- `joint_acceleration`
- `tool_frame`
- `circ_type` (`interim` or `center` for CIRC)
- `circ_point`

## Quick Client Example

You can use the repository sandbox scripts from workspace root, for example:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python sandbox_lin.py
python sandbox_ptp.py
python sandbox_circ.py
```

These scripts submit `ExecuteTrajectory` goals to
`movement_controller/execute_trajectory` and print feedback/results.

## Parameters

The node declares and validates these parameter groups:

- `moveit_group_name`
- `moveit_connection_timeout`
- Workspace bounds:
	- `constraints.workspace.x_min`, `x_max`
	- `constraints.workspace.y_min`, `y_max`
	- `constraints.workspace.z_min`, `z_max`
- Joint constraints:
	- `constraints.joint.names`
	- `constraints.joint.lower_limits`
	- `constraints.joint.upper_limits`
- Orientation tolerances:
	- `constraints.orientation.tolerance_x`
	- `constraints.orientation.tolerance_y`
	- `constraints.orientation.tolerance_z`
- Speed/acceleration caps:
	- `constraints.max_cartesian_speed`
	- `constraints.max_cartesian_acceleration`
	- `constraints.max_joint_speed`
	- `constraints.max_joint_acceleration`

Default speed and joint constraint values are loaded from files in `config/`
through the launch setup.

## Speed and Acceleration Notes

- Joint speed/acceleration are constrained by the strictest joint limits used
	in configuration.
- Cartesian speed behavior depends on both translational and rotational
	components. If rotational limits are tight, observed translation speed may be
	lower than requested on mixed-rotation motions.
- MoveIt Pilz interfaces expose acceleration but not a separate deceleration
	setting in this flow; plan accordingly.

## Testing

Run package tests:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select movement_controller
colcon test-result --verbose
```

You can also run pytest directly for quick iteration:

```bash
python -m pytest src/movement_controller/tests/unit -v
python -m pytest src/movement_controller/tests/integration -v
```

## Troubleshooting

- Action server unavailable:
	- Ensure lifecycle node is `active`.
	- Verify `move_group` and UR driver launched successfully.
- Planning service unavailable:
	- Confirm `plan_sequence_path` exists:
		`ros2 service list | grep plan_sequence_path`
- No motion / immediate rejection:
	- Check path IDs are valid UUID strings.
	- Check `motion_type` values are valid (`LIN`, `PTP`, `CIRC`).
	- Validate goal values against configured workspace/joint/speed constraints.
- Cancellation behavior:
	- The implementation includes a MoveIt workaround by publishing `"stop"` to
		`/trajectory_execution_event` due to known MoveIt cancellation limitations.

## License

BSD-3-Clause

