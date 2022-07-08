cmake_minimum_required(VERSION 3.8)
project(gpn_pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gpn_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(gpn_pid_controller SHARED
  src/gpn_pid_controller.cpp
)
target_compile_definitions(gpn_pid_controller
  PRIVATE "GPN_PID_CONTROLLER_NODE_BUILDING_DLL"
)
ament_target_dependencies(gpn_pid_controller 
  tf2
  rclcpp
  gpn_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  irobot_create_msgs
)

add_executable(gpn_pid_controller_node src/gpn_pid_controller_node.cpp)
target_link_libraries(gpn_pid_controller_node
  gpn_pid_controller
)
ament_target_dependencies(gpn_pid_controller_node 
  rclcpp
)

install(TARGETS
  gpn_pid_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gpn_pid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
