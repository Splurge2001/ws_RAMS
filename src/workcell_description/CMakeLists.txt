cmake_minimum_required(VERSION 3.10)
project(workcell_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch materials
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
