cmake_minimum_required(VERSION 3.8)
project(pmr_tp2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)      # LaserScan
find_package(geometry_msgs REQUIRED)    # Twist, Pose
find_package(nav_msgs REQUIRED)         # Odometry 
find_package(visualization_msgs REQUIRED)  # Marker
find_package(std_msgs REQUIRED)         # Bool
find_package(eigen3_cmake_module REQUIRED)         # Odometry 

find_package(Eigen3 REQUIRED)

# Executables
include_directories(include)

add_executable(tangent_bug
  src/tangent_bug.cpp
)
ament_target_dependencies(tangent_bug
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)

add_executable(parametric_curve
  src/parametric_curve.cpp
)
ament_target_dependencies(parametric_curve
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  Eigen3
)

add_executable(potential_function
  src/potential_function.cpp
)
ament_target_dependencies(potential_function
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)

add_executable(path_with_potential
  src/path_with_potential.cpp
)
ament_target_dependencies(path_with_potential
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  Eigen3
)

# Install
install(TARGETS 
  tangent_bug
  parametric_curve
  potential_function
  path_with_potential
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  description
  config
  rviz
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
