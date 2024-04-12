cmake_minimum_required(VERSION 3.8.0)
project(omtp_lecture5_2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)


# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Used to specify directories to search for header files. It is added to a list of directories to be searched when looking for include files that the compiler uses. 
include_directories(
  include
  ${ament_INCLUDE_DIRECTORIES}
)


# Install the necessary folders in the package share directory. 
install(DIRECTORY launch urdf config 
  DESTINATION share/${PROJECT_NAME}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )


# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES detectron2_ros
# #  CATKIN_DEPENDS rospy
# #  DEPENDS system_lib
# )

ament_package()