cmake_minimum_required(VERSION 3.5)
project(mcqueen_description)

find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch
install(DIRECTORY
  urdf
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
