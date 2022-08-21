cmake_minimum_required(VERSION 3.8)
project(tcl_reference_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tcl_reference_interfaces REQUIRED)

include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(spin_some
src/spin_some_node.cpp)

# add_library(source SHARED
#   src/source_node.cpp)

ament_target_dependencies(spin_some
  "rclcpp"
  "rclcpp_components"
  "tcl_reference_interfaces"
)

add_library(timer SHARED
  src/timer_node.cpp)

ament_target_dependencies(timer
  "rclcpp"
  "rclcpp_components"
  "tcl_reference_interfaces")

rclcpp_components_register_node(timer
  PLUGIN "tcl_reference_system::TimerNode"
  EXECUTABLE timer_node)


add_library(spin SHARED
  src/spin_node.cpp)

ament_target_dependencies(spin
  "rclcpp"
  "rclcpp_components"
  "tcl_reference_interfaces")
  
rclcpp_components_register_node(spin
  PLUGIN "tcl_reference_system::SpinNode"
  EXECUTABLE spin_node)

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

install(TARGETS
  spin_some
  timer
  spin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
