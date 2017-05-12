#!/usr/bin/env python  
import roslib
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/laser_link', rospy.Time(0))
            print trans
        except (tf.LookupException, tf.ConnectivityException):
            continue

        rate.sleep()
