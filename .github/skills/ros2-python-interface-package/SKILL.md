---
name: ros2-python-interface-package
description: "Build a single ament_cmake package that contains BOTH Python code (ament_cmake_python) AND generated ROS 2 interfaces (rosidl_generate_interfaces — .msg/.srv/.action). Resolves the duplicate-target CMake error (ament_cmake_python_symlink_<pkg> / _egg already exists) using the SKIP_INSTALL pattern, installs the generated Python bindings + C/C++ typesupport libraries manually, and optionally installs the interface definitions so `ros2 interface show` works."
argument-hint: "[package_name] — the ament_cmake package that needs both Python + interfaces"
allowed-tools: Read, Write, Edit, Bash, Glob, Grep
---

<objective>
Correctly configure one `ament_cmake` package that ships **both**:

1. A Python module / nodes (installed via `ament_python_install_package`), and
2. Generated ROS 2 interfaces — `.msg`, `.srv`, and/or `.action` files
   (generated via `rosidl_generate_interfaces`).

This is the required layout in this project: per the copilot instructions, ROS 2
interfaces live in the **same** package that implements the node using them (no
separate `*_interfaces` package). That forces Python + `rosidl` into one
`ament_cmake` package, which triggers a known CMake target-name collision that
must be worked around.
</objective>

<symptom>
Naively calling both `rosidl_generate_interfaces(${PROJECT_NAME} ...)` and
`ament_python_install_package(${PROJECT_NAME})` in the same `CMakeLists.txt`
fails at configure time with:

```
CMake Error at .../ament_cmake_python/cmake/ament_python_install_package.cmake:106 (add_custom_target):
  add_custom_target cannot create target "ament_cmake_python_symlink_<pkg>"
  because another target with the same name already exists.
...
  add_custom_target cannot create target "ament_cmake_python_build_<pkg>_egg"
  because another target with the same name already exists.
```

Root cause: `rosidl_generate_interfaces` internally calls
`ament_python_install_package` (to install the generated `_py` bindings). When
you *also* call it for your own Python module, both invocations try to create
the same `ament_cmake_python_*_<pkg>` custom targets → collision.
See: https://github.com/ros2/rosidl_python/issues/141
</symptom>

<fix>
Pass `SKIP_INSTALL` to `rosidl_generate_interfaces` so it does NOT call
`ament_python_install_package` itself, then install the artifacts it skipped
manually. Reference implementation already in this repo:
`src/movement_controller/CMakeLists.txt` and `src/laser_sensors/CMakeLists.txt`.

### 1. CMakeLists.txt

```cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Python version — needed to compute the correct site-packages install path
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# find_package every message package used inside your interface files
find_package(std_msgs REQUIRED)          # example DEPENDENCY

# --------------------------------------------------------------------------
# ROS 2 interface generation
# SKIP_INSTALL: prevents rosidl from calling ament_python_install_package
# internally, which would collide with our own call below.
# See: https://github.com/ros2/rosidl_python/issues/141
# --------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BeamTriggered.msg"
  # "srv/DoThing.srv"
  # "action/ExecuteTrajectory.action"
  DEPENDENCIES std_msgs
  SKIP_INSTALL
)

# --------------------------------------------------------------------------
# Python package installation (your own module: import <pkg>...)
# --------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# Install the generated Python interface bindings that SKIP_INSTALL skipped.
# rosidl_generator_py writes them to ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/.
# Evaluated at install time (after build), so the directory exists.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

# Install the generated C/C++ typesupport shared libraries that SKIP_INSTALL
# skipped. Required at runtime by ANY node (Python or C++) using these interfaces.
install(
  TARGETS
    ${PROJECT_NAME}__rosidl_generator_c
    ${PROJECT_NAME}__rosidl_typesupport_c
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_c
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
    ${PROJECT_NAME}__rosidl_typesupport_introspection_c
    ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# OPTIONAL — install interface definitions into share/ so `ros2 interface show`
# and other tooling can read them (SKIP_INSTALL suppresses this too).
# The .idl is what `ros2 interface show` reads; the source .msg matches the
# standard rosidl package layout. The .idl is generated under rosidl_adapter/.
install(
  FILES "msg/BeamTriggered.msg"
  DESTINATION "share/${PROJECT_NAME}/msg"
)
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${PROJECT_NAME}/msg/BeamTriggered.idl"
  DESTINATION "share/${PROJECT_NAME}/msg"
)
```

### 2. package.xml

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<!-- <depend> for every message pkg referenced in your interface files -->
<depend>std_msgs</depend>

<exec_depend>rclpy</exec_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

<!-- Marks this as an interface package so rosidl typesupport resolves it -->
<member_of_group>rosidl_interface_packages</member_of_group>

<export>
  <build_type>ament_cmake</build_type>
</export>
```

### 3. Directory layout

```
<pkg>/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── BeamTriggered.msg
├── srv/                     # optional
├── action/                  # optional
└── <pkg>/                   # the Python module (ament_python_install_package)
    ├── __init__.py
    └── my_node.py
```
</fix>

<pitfalls>
- **Every message package** referenced inside a `.msg`/`.srv`/`.action` needs
  BOTH a `find_package(<pkg> REQUIRED)` in CMake AND appears in the
  `DEPENDENCIES` list of `rosidl_generate_interfaces`, AND a `<depend>` in
  package.xml.
- **Do not** forget `<member_of_group>rosidl_interface_packages</member_of_group>`
  — without it, downstream typesupport resolution can fail.
- The generated typesupport target names are literal and must be listed exactly
  as `${PROJECT_NAME}__rosidl_*`. If a future ROS distro changes this set, list
  what actually exists under `build/<pkg>/` after a build.
- `SKIP_INSTALL` also suppresses installing the `.idl`/`.msg` into `share/`, so
  `ros2 interface show <pkg>/msg/Foo` will report "Could not find the interface"
  unless you add the optional `install(FILES ...)` rules above. Runtime pub/sub
  and Python imports work WITHOUT them — they are only needed for introspection
  tooling.
- The `.idl` is produced under
  `build/<pkg>/rosidl_adapter/<pkg>/msg/Foo.idl` — confirm the exact path with
  `find build/<pkg> -name "Foo.idl"` before writing the install rule.
- `--symlink-install` does not symlink generated interface bindings; you must
  rebuild the package after editing any `.msg`/`.srv`/`.action` file.
</pitfalls>

<verify>
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash

# Python bindings import (works even without the optional share/ install):
python3 -c "from <pkg>.msg import BeamTriggered; m = BeamTriggered(); print('OK', m)"

# Introspection tooling (only works if the optional install(FILES ...) was added):
ros2 interface show <pkg>/msg/BeamTriggered
```
Success = build finishes with no duplicate-target error, the Python import
succeeds, and (if the optional rules were added) `ros2 interface show` prints
the definition.
</verify>
