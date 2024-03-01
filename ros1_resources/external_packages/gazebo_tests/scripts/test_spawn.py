#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Point, Quaternion


sdf_model = """<sdf version="1.4">
  <model name="cube">
    <static>0</static>
    <link name="link">

      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.00001066664</ixx>
          <ixy>0.000000</ixy>
          <ixz>0.000000</ixz>
          <iyy>0.00001066664</iyy>
          <iyz>0.000000</iyz>
          <izz>0.00001066664</izz>
        </inertia>
      </inertial>

      <collision name="colision1">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>1.0 1.0 1.0</size>
          </box>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <fdir1>1 1 1</fdir1>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000.0</kp>
              <kd>1.0</kd>
              <min_depth>0.001</min_depth>
              <max_vel>0.1</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual1">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>1.0 1.0 1.0</size>
          </box>
        </geometry>
      </visual>

      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""


if __name__ == '__main__':
    rospy.init_node('test_spawn')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_srv.wait_for_service()
    req = SpawnModelRequest()
    req.model_name = 'test_model'
    req.model_xml = sdf_model
    req.robot_namespace = ""
    req.reference_frame = ""
    req.initial_pose = Pose()
    req.initial_pose.position.z = 1.0
    #req.initial_pose.orientation.w = 1.0
    rospy.loginfo("Call: " + str(req))
    resp = spawn_srv.call(req)
    rospy.loginfo("Response: " + str(resp))
