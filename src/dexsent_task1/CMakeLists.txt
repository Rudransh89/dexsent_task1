cmake_minimum_required(VERSION 3.8)
project(dexsent_task1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install the directories (THIS WAS MISSING)
install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
