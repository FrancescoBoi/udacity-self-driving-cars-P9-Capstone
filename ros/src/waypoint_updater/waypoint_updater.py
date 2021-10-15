#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        #self.base_lane = None
        self.stop_line_wp_idx = -1
        # use loop instead of spin for control over publishing frequency
        #rospy.spin()
        self.loop()

    def loop(self):
        # the resulting freq will be prob ~30Hz: WP goes to WP_follower,
        # a component of Autoware running at 30Hz
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                #get closest WP
                closest_wp_idx = self.get_closest_wp_idx()
                self.publish_wp(closest_wp_idx)
            rate.sleep()

    def get_closest_wp_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # .query([...], numOfEls2BeReturned)
        # result of query: [???, idx]
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # check if the closest WP is ahead or behind:
        closest_wp = self.waypoints_2d[closest_idx]
        prev_wp = self.waypoints_2d[closest_idx-1]

        #equation for hyperplane through closest_wp
        #closest_wp- prev_wp is a vecotr that defines a hyperplane perpendicular
        # to it
        # the car is either behind or ahead of this wp. If it is ahead, then
        # the vector position-closest_wp  points towards the same direction
        # of the vector closest_wp- prev_wp, otherwise they have opposite direction
        # If they point to the same direction then the dot product is >0
        # If so, the ahead WP is actually the next one
        cl_vect = np.array(closest_wp)
        prev_vect = np.array(prev_wp)
        pos_vect = np.array([x,y])
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val >0:
            closest_idx = (closest_idx+1)%len(self.waypoints_2d)
        return closest_idx

    def publish_wp(self, closest_idx):
        final_lane = self.generate_lane()
        #lane.header = self.base_waypoints.header
        #lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        closest_idx = self.get_closest_wp_idx()
        furthest_idx = closest_idx+ LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:furthest_idx]
        if self.stop_line_wp_idx== -1 or (self.stop_line_wp_idx>= furthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # -2: to stop a lil before the line
            stop_idx = max(self.stop_line_wp_idx-closest_idx-2, 0)
            dist = self.distance(waypoints, i, stop_idx)#linear piecewise sum
            # dist=0 if i>stop_idx
            vel = math.sqrt(2*MAX_DECEL*dist)#decreasing parabola
            if vel<1.:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x) #
            #sqrt can be large for large dist: keep the vel given by the WP
            temp.append(p)
        return temp

    #50Hz
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    # latched callback: called just ones
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        # use KDTree: logN instead of N complexity
        #convert the waypoints to 2D coordinates:
        if self.waypoints_2d is None:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_line_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
