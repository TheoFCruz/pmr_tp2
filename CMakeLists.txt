cmake_minimum_required(VERSION 3.8)
project(pmr_tp2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)    # Twist, Pose
find_package(nav_msgs REQUIRED)         # Odometry, OccupancyGrid
find_package(visualization_msgs REQUIRED)  # Marker
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 REQUIRED)

# Executables
include_directories(include)

add_executable(astar
  src/astar.cpp
)
ament_target_dependencies(astar
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)

add_executable(gvd
  src/gvd.cpp
)
ament_target_dependencies(gvd
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)

add_executable(rrt
  src/rrt.cpp
)
ament_target_dependencies(rrt
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)

# Install
install(TARGETS 
  astar
  gvd
  rrt
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
