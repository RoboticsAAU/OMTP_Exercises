cmake_minimum_required(VERSION 3.8)
project(omtp_lecture3_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ariac_sensors REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(logic_pick_and_place src/logic_pick_and_place.cpp)
ament_target_dependencies(logic_pick_and_place rclcpp moveit_ros_planning_interface moveit_visual_tools ariac_msgs)


install(TARGETS
  logic_pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

# rosidl_get_typesupport_target(cpp_typesupport_target "ariac_msgs" rosidl_type_support_cpp)

# target_link_libraries(logic_pick_and_place
#   ${cpp_typesupport_target}
# )

ament_package()
