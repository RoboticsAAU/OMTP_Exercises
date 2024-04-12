cmake_minimum_required(VERSION 3.8.0)
project(depth_camera_gazebo_plugin)

# The package must have .cmake binary files in the directories for the find_package to work, otherwise it will need the full path to the package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# catkin_package(
#     DEPENDS
#         roscpp
#         gazebo_ros
#         image_transport
#         camera_info_manager
#         sensor_msgs
# )


message("GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")



include_directories(include ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
    ${PROJECT_NAME}
    src/DepthCameraPlugin.cpp
    src/gazebo_ros_depth_camera.cpp
    )


target_link_libraries(${PROJECT_NAME} ${ament_LIBRARIES} ${GAZEBO_LIBRARIES})

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
)

# Install Targets
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# Install other files if needed

# install(
#     TARGETS
#         ${PROJECT_NAME}
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# foreach(dir include)
#     install(DIRECTORY ${dir}/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)
