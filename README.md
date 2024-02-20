# Exercise 1 
The first exercise has been conducted. Since the ros packages are given in ROS1 and this is ported to ROS2, therefore most of the time has been spend on porting the `external_packages` to ROS2. Besides porting every package, this documentation will only accompany the changes made with regards to the exercises.

## Exercise 1.1 2x URDF tutorials
Due to sufficient knowledge of Universal Robot Description Format (URDF) files, I decided to skip this exercise. 

## Exercise 1.2 XACRO tutorials
Due to sufficient experience with xacro and it's capabilities, i decided to skip this exercise. 

## Exercise 1.3 Inspect the OMTP factory


# Exercise 2 
The main objective of this exercise is to create a moveit_config package and then setup the simulation in gazebo, such that the manipulator trajectories are executed in the physics engine and visualized in rviz. 

By default, moveit2 only generates launch files for rviz only, this entails that there is no physics simulation. Therefore the first task is to get gazebo up and running with rviz. To do that, I configured the moveit_config with moveit_setup_assistant. This is done initially without an effort controller, as I saw problems with the effort controllers. After I configured the moveit_package, I added a package called `omtp_lecture2_2` that includes the necessary resource files from the external packages. This is needed for gazebo to visualize the mesheses of the spawned objects. This is included in the `package.xml` file

The launch script `gazebo.launch.py` is located in the package mentioned above, and it spawns the following nodes:

- `static_virtual_joints_tfs` which is all the static joints in the urdf file.
- `robot_state_publisher` this node publishes all the robot states including link information, which is more than just the joint state, which is only the joint states. 
- `move_group_node` This node is responsible for the fusion of robot states, controller etc. It orchestrates everything regarding the trajectory generation and makes sure that it can plan and execute trajectories. 
- `rviz` this spawns rviz with the moveit_config panels and configuration. 
- `spawn_controllers` spawns the controllers for the manipulator. 
- `gazebo` spawns gazebo with the necessary files.
- `spawn_entity` , parses the robot_description into the .sdf format internally, and thus integrates it into the gazebo world. 

It is important to note that the gazebo_ros2_control plugin must be placed as a hardware plugin into `omtp_factory.ros2_control.xacro` and the name in `omtp_factory.urdf.xacro` must be changed to GazeboSystem. This is crucial to tell moveit that the robot is simulated in gazebo. 

## Spawn Box
Spawning the green box is done using the launch script
```bash
ros2 launch omtp_lecture2_2 spawn_box.launch.py
```
which spawns the box in the world reference frame. I tried to make it spawn in the `module1_festo_module` reference frame, but due to the automatic parsing of the `.urdf` file to `.sdf`, then static frames are disregarded, and it is therefore not visible in gazebo. The box is however spawned on top of the festo module using the following coordinates in the `box.sdf` file: 
```xml
<pose>-1.08 2.55 1.0 0 0 0</pose>
```

which puts it directly on the conveyor belt. 


## pick and place 
Using the C++ moveit2 api, it is possible to control the movegroup by setting positions and targets. It is possible to request the move_group_interface to plan trajectories between the targets for both move_groups. (`panda_arm` and `hand`) 
Positions are defined with the standard `geometry_msgs::msg::Pose`, that is, it needs a translational position and a position defined in quaternions. 

The pick and place algorithm is in the `omtp_lecture2_2/src/move_demo.cpp`. To run the pick and place with a box, use the `omtp_lecture2_2/launch/pick_and_place.launch` file. This launches the full gazebo simulation, the move demo node, and spawns a box in the environment.



