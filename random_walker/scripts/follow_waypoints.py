#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi

class Follow_WayPoints():
    def __init__(self):
        rospy.init_node('follow_waypoints', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        quaternions = list()
        poses = list()
        waypoints = list()
      
        pose_list = [[ 54.5, -10.9, -2.7],
                     [-15.1,  -9.3,  0.2],
                     [ 14.0, -12.6, -1.6],
                     [ -4.6, -12.8,  1.3],
                     [ 30.4, -15.2, -2.7],
                     [  0.9,  -9.6,  1.3]]
       
        for row in pose_list:
            q_angle = quaternion_from_euler(0.0, 0.0, row[2])
            q = Quaternion(*q_angle)
            p = Point(row[0], row[1], 0.0)
            quaternions.append(q)
            poses.append(p)
            waypoints.append(Pose(p, q))
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        i = 0
        
        while i < len(waypoints) and not rospy.is_shutdown():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = waypoints[i]
            self.move(goal)
            i += 1
        
    def move(self, goal):
            self.move_base.send_goal(goal)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(1200)) 
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Follow_WayPoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
