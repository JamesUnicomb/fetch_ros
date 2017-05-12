#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_python import MoveGroupInterface, PlanningSceneInterface

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import radians, pi
import numpy as np

from std_msgs.msg import String

def main():
    rospy.init_node('move_group_python_path_follow',anonymous=True)

    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = MoveGroupInterface("arm", "base_link")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    print "============ Robot Groups:"
    print robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    gripper_frame = 'wrist_roll_link'

    gripper_poses = list()
    radius = 0.2
    for theta in np.arange(-pi, pi, 0.5 * pi):
        gripper_poses.append(Pose(Point(0.75, radius * np.sin(theta), 0.6 + radius * np.cos(theta)),
                                Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))))

    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = gripper_poses[0]

    move_group.moveToPose(gripper_pose_stamped, gripper_frame)
    result = move_group.get_move_action().get_result()

    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Success!")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())

    trajectory = RobotTrajectory()

    (plan, fraction) = group.compute_cartesian_path(gripper_poses, 0.01, 0.0)

    group.execute(plan)
    move_group.get_move_action().cancel_all_goals()
    

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
