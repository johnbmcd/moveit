#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Quaternion
from math import pi
from moveit_msgs import msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

def closeGripper(pre_grasp_posture):
    pre_grasp_posture.joint_names.append('panda_finger_joint1')
    pre_grasp_posture.joint_names.append('panda_finger_joint2')
    pt = JointTrajectoryPoint()
    pt.positions.append(0.0)
    pt.positions.append(0.0)
    pt.time_from_start = rospy.Duration(0.5)
    pre_grasp_posture.points.append(pt)

def openGripper(grasp_posture):
    grasp_posture.joint_names.append('panda_finger_joint1')
    grasp_posture.joint_names.append('panda_finger_joint2')
    pt = JointTrajectoryPoint()
    pt.positions.append(0.04)
    pt.positions.append(0.04)
    pt.time_from_start = rospy.Duration(0.5)
    grasp_posture.points.append(pt)

def pick(move_group):
    # pick an object
    grasp = msg.Grasp()
    q = quaternion_from_euler(-pi/2, -pi/4, -pi/2)
    grasp.grasp_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    grasp.grasp_pose.pose.position.x = 0.415
    grasp.grasp_pose.pose.position.y = 0
    grasp.grasp_pose.pose.position.z = 0.5

    grasp.pre_grasp_approach.direction.header.frame_id = 'panda_link0'
    grasp.pre_grasp_approach.direction.vector.x = 1.0
    grasp.pre_grasp_approach.min_distance = 0.095
    grasp.pre_grasp_approach.desired_distance = 0.115

    grasp.post_grasp_retreat.direction.header.frame_id = 'panda_link0'
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.1
    grasp.post_grasp_retreat.desired_distance = 0.25

    openGripper(grasp.pre_grasp_posture)

    closeGripper(grasp.grasp_posture)

    move_group.set_support_surface_name('table1')

    move_group.pick('object',grasp)

def addCollisionObjects(planning_scene_interface):
    # clean the scene
    planning_scene_interface.remove_world_object("table1")
    planning_scene_interface.remove_world_object("table2")
    planning_scene_interface.remove_world_object("object")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = 'panda_link0'
    p.pose.position.x = 0.5
    p.pose.position.y = 0
    p.pose.position.z = 0.2
    p.pose.orientation.w = 1.0
    planning_scene_interface.add_box("table1", p, (0.2, 0.4, 0.4))

    p.pose.position.x = 0
    p.pose.position.y = 0.5
    p.pose.position.z = 0.2
    planning_scene_interface.add_box("table2", p, (0.4, 0.2, 0.4))

    p.pose.position.x = 0.5
    p.pose.position.y = 0
    p.pose.position.z = 0.5
    planning_scene_interface.add_box("object", p, (0.02, 0.02, 0.2))

    rospy.sleep(2)

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()

    group = robot.get_group('panda_arm')
    group.set_planning_time(seconds=45.0)

    rospy.sleep(2)

    addCollisionObjects(scene)
    
    rospy.sleep(1)

    pick(group)

    rospy.spin()
    roscpp_shutdown()
