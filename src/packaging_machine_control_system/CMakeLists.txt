cmake_minimum_required(VERSION 3.8)
project(packaging_machine_control_system)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(smdps_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(canopen_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(packaging_machine_manager src/manager.cpp)
target_link_libraries(packaging_machine_manager)
ament_target_dependencies(packaging_machine_manager 
  rclcpp 
  rclcpp_action
  rclcpp_components
  rcl_interfaces
  composition_interfaces
  smdps_msgs)

add_executable(packaging_machine_node src/packaging_machine_node.cpp)
target_link_libraries(packaging_machine_node)
ament_target_dependencies(packaging_machine_node 
  std_msgs
  std_srvs
  rclcpp 
  rclcpp_action
  rclcpp_components
  smdps_msgs
  canopen_interfaces
)

add_library(packaging_machine_action_client SHARED
  src/packaging_machine_action_client.cpp)
target_compile_definitions(packaging_machine_action_client
  PRIVATE "PACKAGING_MACHINE_ACTION_CLIENT_BUILDING_DLL")
ament_target_dependencies(packaging_machine_action_client
  rclcpp
  rclcpp_action
  rclcpp_components
  smdps_msgs)
rclcpp_components_register_nodes(packaging_machine_action_client "action_client::PackagingMachineActionClient")
# rclcpp_components_register_nodes(packaging_machine_action_client PLUGIN 
#   "packaging_machine_action_client_cpp::packaging_machine_action_client" 
#   EXECUTABLE packaging_machine_action_action_client_exe)

# rclcpp_components_register_nodes(packaging_machine_action_client "composition::PackagingMachineActionClient")
# set(node_plugins "${node_plugins}composition::PackagingMachineActionClient;$<TARGET_FILE:packaging_machine_action_client>\n")

install(TARGETS
  packaging_machine_manager
  packaging_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  packaging_machine_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
