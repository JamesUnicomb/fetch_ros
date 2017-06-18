#!/usr/bin/env python
import os, sys, time

# wave.py: "Wave" the fetch gripper
import rospy
import tf
import tf_conversions
import numpy as np
import PyKDL
import actionlib
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from math import radians, pi
from geometry_msgs.msg import Pose, Quaternion, Point

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node('ar_detect_and_grab')

    listener = tf.TransformListener()

    try:
        listener.waitForTransform('/base_link', 'handle_3', rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/base_link', '/handle_3', rospy.Time(0))
        res = Pose(Point(*trans), Quaternion(*rot))
        frame_gripper_link = PyKDL.Frame(PyKDL.Vector(-0.166,0.0,0.0))
        (trans_res, rot_res) = tf_conversions.posemath.toTf(tf_conversions.posemath.fromMsg(res) * frame_gripper_link)
        frame_prep_link = PyKDL.Frame(PyKDL.Vector(-0.35,0.0,0.0))
        (trans_prep, rot_prep) = tf_conversions.posemath.toTf(tf_conversions.posemath.fromMsg(res) * frame_prep_link)
    except (tf.LookupException, tf.ConnectivityException):
        print 'couldnt get transform to handle'

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    gripper_group = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    close_command = GripperCommandGoal()
    close_command.command.max_effort = 60
    close_command.command.position = 0.0

    open_command = GripperCommandGoal()
    open_command.command.max_effort = 60
    open_command.command.position = 0.095

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'
    # Position and rotation of two "wave end poses"
    gripper_actions = [(Pose(Point(*trans_prep), Quaternion(*rot_prep)), open_command),
                        (Pose(Point(*trans_res), Quaternion(*rot_res)), close_command),
                        (Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657)), close_command),
                        (Pose(Point(*trans_res), Quaternion(*rot_res)), close_command),
                        (Pose(Point(*trans_res), Quaternion(*rot_res)), open_command),
                        (Pose(Point(*trans_prep), Quaternion(*rot_prep)), open_command)]
    
    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    for pose, goal in gripper_actions:
        # Finish building the Pose_stamped message
        # If the message stamp is not current it could be ignored
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        # Set the message pose
        gripper_pose_stamped.pose = pose
        # set the gripper grasp

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()

        if result:
            gripper_group.send_goal(goal)
            time.sleep(3)
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
