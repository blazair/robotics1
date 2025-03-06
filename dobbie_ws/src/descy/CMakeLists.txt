cmake_minimum_required(VERSION 3.8)
project(descy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

# Install directories
install(
  DIRECTORY launch meshes urdf rviz scripts worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Add Python executable
install(PROGRAMS
  scripts/gui.py
  scripts/tesseract.py
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
