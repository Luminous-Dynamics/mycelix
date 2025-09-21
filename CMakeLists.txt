cmake_minimum_required(VERSION 3.8)
project(mycelix_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Create libraries for reusable components
add_library(holochain_agent_lib
  src/holochain_agent.cpp
)

add_library(federated_learner_lib
  src/federated_learner.cpp
)

# Add executables
add_executable(mycelix_bridge_node 
  src/mycelix_bridge_node.cpp
  src/safety_validator.cpp
)

add_executable(swarm_coordinator
  src/swarm_coordinator.cpp
)

add_executable(consciousness_visualizer
  src/consciousness_visualizer.cpp
)

add_executable(robot_simulator
  src/robot_simulator.cpp
)

add_executable(demo_controller
  src/demo_controller.cpp
)

# Link libraries for holochain_agent_lib
ament_target_dependencies(holochain_agent_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Link libraries for federated_learner_lib
ament_target_dependencies(federated_learner_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Link libraries for mycelix_bridge_node
target_link_libraries(mycelix_bridge_node
  holochain_agent_lib
  federated_learner_lib
)
ament_target_dependencies(mycelix_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Link libraries for swarm_coordinator
ament_target_dependencies(swarm_coordinator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# Link libraries for consciousness_visualizer
ament_target_dependencies(consciousness_visualizer
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
)

# Link libraries for robot_simulator
ament_target_dependencies(robot_simulator
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
)

# Link libraries for demo_controller
ament_target_dependencies(demo_controller
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  mycelix_bridge_node
  swarm_coordinator
  consciousness_visualizer
  robot_simulator
  demo_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  holochain_agent_lib
  federated_learner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()