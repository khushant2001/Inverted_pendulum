cmake_minimum_required(VERSION 3.8)
project(config_files)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install configuration files
install(
  DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
