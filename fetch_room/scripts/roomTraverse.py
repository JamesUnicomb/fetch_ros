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
from visualization_msgs.msg import Marker

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('fetch_room')


class RoomCheck:
    def __init__(self):
        loc_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.room_pub_cb)

        self.room_pub = rospy.Publisher('/fetch_room', String, queue_size=5)
        self.marker_pub = rospy.Publisher('/waypoint_markers', Marker)

        f = open(pkg_path + '/maps/room_code.yaml')
        map_info = yaml.load(f)

        self.img_array = cv2.imread(pkg_path + '/maps/' + map_info['image'])

        x_origin, y_origin, theta_origin = map_info['origin']

        self.ny, self.nx, rgb = self.img_array.shape
        res = map_info['resolution']
        self.room_codings = map_info['room_codings']
        self.room_graph = map_info['node_connections']
        self.room_transitions = map_info['transition_data']

        self.x_arr = np.arange(x_origin, self.nx * res + x_origin, res)
        self.y_arr = np.arange(y_origin, self.ny * res + y_origin, res)

        self.current_room = 404


    def find_key(self, input_dict, value):
        return next(k for k, v in input_dict.items() if (v == value).all())


    def room_from_xy(self, x, y):
        x_ind = next(i for i,v in enumerate(self.x_arr) if v > x)
        y_ind = next(i for i,v in enumerate(self.y_arr) if v > y)

        query_room = self.find_key(self.room_codings, self.img_array[self.ny - y_ind][x_ind])

        return query_room


    def room_pub_cb(self, pose):
        x, y = pose.pose.pose.position.x, pose.pose.pose.position.y
        try:
            self.current_room = self.room_from_xy(x,y)

            if self.current_room==None:
                str_pub = 'fetch is in room: NONE'
            else:
                str_pub = 'fetch is in room: %d' % (self.current_room)

            self.room_pub.publish(str_pub)

        except (AttributeError):
            rospy.loginfo('Node Initialising...')
            rospy.loginfo('Wating for AMCL Pose data...')


    def bfs_paths(self, graph, start, goal):
        queue = [(start, [start])]
        while queue:
            (vertex, path) = queue.pop(0)
            for next in set(graph[vertex]) - set(path):
                if next == goal:
                    yield path + [next]
                else:
                    queue.append((next, path + [next]))


    def shortest_path(self, graph, start, goal):
        try:
            return next(self.bfs_paths(graph, start, goal))
        except StopIteration:
            return None


    def room_path(self, start, goal):
        return self.shortest_path(self.room_graph, start, goal)

   
    def action_list(self, end_pose):
        start_room = self.current_room
        end_room = self.room_from_xy(end_pose.position.x, end_pose.position.y)

        room_list = self.room_path(start_room, end_room)

        door_actions = []
        pose_list = []

        for i in range(len(room_list) - 1):
            in_room = room_list[i]
            out_room = room_list[i+1]
            door_pose = self.room_transitions[in_room][out_room]
            door_actions.append(door_pose['door_action'])
            q = quaternion_from_euler(0.0, 0.0, door_pose['theta'])
            pose_list.append(Pose(Point(door_pose['x'], door_pose['y'], 0.0), 
                                             Quaternion(*q)))

        door_actions.append('None')
        pose_list.append(end_pose)

        return pose_list, door_actions

    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.SPHERE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()





class FetchBase:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))


class openDoors:
    def __init__(self):
        self.open_doors = False
