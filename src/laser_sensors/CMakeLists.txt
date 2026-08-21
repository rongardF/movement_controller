cmake_minimum_required(VERSION 3.8)
project(laser_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install the Python module so it is importable (import laser_sensors...).
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch urdf srdf model
  DESTINATION share/${PROJECT_NAME}
)

# Install the Captron ORL2 node so it is discoverable as
# `ros2 run laser_sensors captron_orl2` and via the launch file.
install(PROGRAMS
  ${PROJECT_NAME}/captron_orl2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME captron_orl2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
