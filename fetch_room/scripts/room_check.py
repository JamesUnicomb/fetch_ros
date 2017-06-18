#!/usr/bin/env python

import rospy, rospkg, yaml
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import numpy as np
import cv2

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('fetch_room')

rospy.init_node('fetch_room')

from roomTraverse import FetchBase, RoomCheck


def main():
    room_checker = RoomCheck()
    fetch_base = FetchBase()

    test_pose = Pose(Point(-27.97, -0.98, 0.0), 
                              Quaternion(0.0, 0.0, 0.722, 0.692))

    pose_list, action_list = room_checker.action_list(test_pose)
    print pose_list

    for pose in pose_list:
        p = Point()
        p = pose.position
        room_checker.markers.points.append(p)

    room_checker.marker_pub.publish(room_checker.markers)

    for pose in pose_list:
        pose_stamped = MoveBaseGoal()
        pose_stamped.target_pose.header.frame_id = '/map'
        pose_stamped.target_pose.header.stamp = rospy.Time.now()
        pose_stamped.target_pose.pose = pose
          


        print pose_stamped

        fetch_base.move_base.send_goal(pose_stamped)

        success = fetch_base.move_base.wait_for_result()
 
        if success:
            break
        

if __name__=="__main__":
    main()

