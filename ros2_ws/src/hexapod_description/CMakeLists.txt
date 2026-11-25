cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz meshes
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gen_legs_yaml.py scripts/check_fk.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
