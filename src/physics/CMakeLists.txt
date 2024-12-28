cmake_minimum_required(VERSION 3.8)
project(physics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY src/model/
  DESTINATION share/${PROJECT_NAME}/model
)

ament_package()
