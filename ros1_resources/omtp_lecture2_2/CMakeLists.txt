cmake_minimum_required(VERSION 3.5)
project(omtp_lecture2)

# Find packages in the ROS 2 ecosystem
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create an executable
add_executable(move_robot src/move_robot.cpp)

# Specify dependencies for the executable
ament_target_dependencies(move_robot rclcpp std_msgs)

# Install the executable
install(TARGETS
  move_robot
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})


# Specify the Python scripts to be installed
install(PROGRAMS
  scripts/move_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create ament index resource which references the libraries in the binary output
ament_export_dependencies(ament_cmake)
ament_package()