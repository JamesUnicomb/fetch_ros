#!/usr/bin/env python

import numpy as np
import rospy
import actionlib
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan, JointState, Image, Joy
from fetch_driver_msgs.msg import GripperState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandGoal, GripperCommandAction
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion

## code is adapted from openai fetch rosbridge



class FetchArmController:
    def __init__(self):
        self.moving_mode = False
        self.plan_only = False
        self.prev_joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.joint_names = ["shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "upperarm_roll_joint",
                            "elbow_flex_joint",
                            "forearm_roll_joint",
                            "wrist_flex_joint",
                            "wrist_roll_joint"]

        self.joy_subscriber = rospy.Subscriber("/fetch_joy", Joy, self.joy_callback)
        self.joints_subscriber = rospy.Subscriber("/joint_states", JointState, self.joints_states_callback)
        self.rgbimage_subscriber = rospy.Subscriber("/head_camera/depth/image", Image, self.depthimage_callback)
        self.depthimage_subscriber = rospy.Subscriber("/head_camera/depth/image", Image, self.rgbimage_callback)

        self.reset_subscriber = rospy.Subscriber("/fetch_ai_arm_reset_pos", Empty, self.arm_reset_callback)
        self.reset_subscriber = rospy.Subscriber("/fetch_ai_arm_start_pos", Empty, self.arm_start_callback)

        self.arm_effort_pub = rospy.Publisher("/arm_controller/weightless_torque/command", JointTrajectory, queue_size=2)
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.arm_cartesian_twist_pub = rospy.Publisher("/arm_controller/cartesian_twist/command", Twist, queue_size=2)
        self.head_point_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.arm_move_group = MoveGroupInterface("arm", "base_link", plan_only = self.plan_only)

        planning_scene = PlanningSceneInterface("base_link")
        planning_scene.removeCollisionObject("my_front_ground")
        planning_scene.removeCollisionObject("my_back_ground")
        planning_scene.removeCollisionObject("my_right_ground")
        planning_scene.removeCollisionObject("my_left_ground")
        planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    def joints_states_callback(self, data):
        if len(data.name) == 13:
            self.arm_joints = data.position[-7:]

    def depthimage_callback(self, image):
        self.depth_image = image

    def rgbimage_callback(self, image):
        self.rgb_image = image

    def joy_callback(self, joy):
        if not self.moving_mode:
            self.arm_control_action(joy)
        self.prev_joy_buttons = joy.buttons

    def arm_control_action(self, joy):
        twist = Twist()
        twist.linear.x = 0.2 * joy.axes[1]
        twist.linear.y = 0.2 * joy.axes[0]
        twist.linear.z = - 0.2 * joy.axes[2]
        twist.angular.x = 0.2 * joy.axes[5]
        twist.angular.y = 0.2 * joy.axes[6]
        twist.angular.z = 0.2 * joy.axes[4]
        self.arm_cartesian_twist_pub.publish(twist)
        
        if joy.buttons[0] and not self.prev_joy_buttons[0]:
            goal = GripperCommandGoal()
            goal.command.max_effort = 60
            goal.command.position = 0.0
            self.gripper_client.send_goal(goal)
        elif not joy.buttons[0] and self.prev_joy_buttons[0]:
            goal = GripperCommandGoal()
            goal.command.max_effort = 60
            goal.command.position = 0.1
            self.gripper_client.send_goal(goal)

    def arm_reset_callback(self, data):
        reset_pose_stamped = PoseStamped()
        reset_pose_stamped.header.frame_id = "base_link"
        reset_pose_stamped.header.stamp = rospy.Time.now()
        reset_pose_stamped.pose = Pose(Point(0.055, -0.140, 0.623), Quaternion(0.460, -0.504, 0.511, 0.523))

        self.arm_move_group.moveToPose(reset_pose_stamped, "wrist_roll_link")

    def arm_start_callback(self, data):
        reset_pose_stamped = PoseStamped()
        reset_pose_stamped.header.frame_id = "base_link"
        reset_pose_stamped.header.stamp = rospy.Time.now()
        reset_pose_stamped.pose = Pose(Point(0.213, -0.520, 0.605), Quaternion(0.545, 0.766, -0.197, 0.278))

        self.arm_move_group.moveToPose(reset_pose_stamped, "wrist_roll_link")
        

def main():
    rospy.init_node("move_group_python",anonymous=True)
    arm_control = FetchArmController()
    rospy.spin()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass      
