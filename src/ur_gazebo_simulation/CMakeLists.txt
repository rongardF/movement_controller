cmake_minimum_required(VERSION 3.5)
project(ur_gazebo_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf world model
  DESTINATION share/${PROJECT_NAME}
)


ament_package()