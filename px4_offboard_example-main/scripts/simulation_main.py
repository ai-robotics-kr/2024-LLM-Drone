#!/usr/bin/env python

import rospy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from offboard_example.msg import WayPoint
from std_msgs.msg import Bool, Int32, Float32
from math import sin, cos, sqrt, pi
import numpy as np

class Mission:
    def __init__(self):
        self.current_state = State()
        self.global_home_position = GeoPoint()
        self.current_local_position = Point()
        
        self.earth_radius = None
        self.home_set = False
        
        # Waypoints and Obstacle point
        self.local_home_position = None
        self.wpTakeoff = None

        self.wp1 = None
        self.wp2 = None
        self.wp3 = None

        self.obstacle = None

        self.step = 0
        # Publisher
        self.local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        #self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/offboard_example/waypoint', WayPoint, queue_size=10)

        # Subscriber
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.LocalPositionCb)
        # Service_client
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    
    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_throttle(1, "Wait FCU connection")

        rospy.loginfo("FCU connected")

    def check_home_setting(self):
        while not self.home_set:
            rospy.loginfo_throttle(1, "Wait Home position Setting")
        rospy.loginfo("Home position is setted")

    def setEarthRadius(self):
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter

        lat = self.global_home_position.latitude

        self.earth_radius = sqrt(((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))

    def setMode(self, mode):
        # Custom Mode List: 
        # http://wiki.ros.org/mavros/CustomModes

        rate = rospy.Rate(0.5)
        while True:
            self.PubLocalPosition(self.local_home_position)

            if self.current_state.mode != mode:
                self.set_mode_client(base_mode=0, custom_mode=mode)
            
            else:
                break

            rate.sleep()
                
    def setArm(self):
        rate = rospy.Rate(0.5)
        while True:
            if self.current_state.armed is not True:
                self.arming_client(True)

            else:
                break

            rate.sleep()

    def setWayPoints(self):
        self.local_home_position = Point(0, 0, 0)

        self.wpTakeoff = Point(0, 0, rospy.get_param('/takeoff_alt', 30.0))

        self.wp1 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp1'))
        self.wp2 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp2'))
        self.wp3 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp3'))
        
        rospy.loginfo('Set Waypoint')

    def setObstaclePoints(self):
        # Z axis means the circle range
        self.obstacle = self.ConvertLocalPoint(rospy.get_param('/obstacle'))

        rospy.loginfo('Set Obstacle')

    def stateCb(self, msg):
        prev_state = self.current_state

        self.current_state = msg
        
        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_state.mode)

        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
        
    def homeCb(self, msg):
        self.home_set = True

        self.global_home_position.latitude = msg.geo.latitude
        self.global_home_position.longitude = msg.geo.longitude

    def LocalPositionCb(self, msg):
        self.current_local_position.x = msg.pose.position.x
        self.current_local_position.y = msg.pose.position.y
        self.current_local_position.z = msg.pose.position.z

    def PubLocalPosition(self, point):
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = point.z

        self.local_pose_pub.publish(pose)

    def PublishVelocity(self, vel):
        self.velocity_pub.publish(vel)

    def PublishWayPoint(self, wp):
        self.waypoint_pub.publish(wp)

    def ConvertLocalPoint(self, global_point):
        local_point = Point()

        home_lat = self.global_home_position.latitude
        home_lon = self.global_home_position.longitude
        
        local_point.x = self.earth_radius * cos(home_lat * pi / 180) * (global_point[1] - home_lon) * pi / 180
        local_point.y = self.earth_radius * (global_point[0] - home_lat) * pi / 180
        local_point.z = global_point[2]

        return local_point

    def waypoint_reach_check(self, wp, reach_range):

        x = self.current_local_position.x
        y = self.current_local_position.y
        z = self.current_local_position.z

        d = sqrt((wp[1].x - x)**2 + (wp[1].y - y)**2 + (wp[1].z - z)**2)

        if d < reach_range :
            return True
        else:
            rospy.loginfo_throttle(1, '%s Process <Distance: %.1f>'%(wp[0], d))
            return False


    def process(self):
        process = {0:['Takeoff', self.wpTakeoff],
                   1:['WP1', self.wp1],
                   2:['WP2', self.wp2],
                   3:['WP3', self.wp3],
                   4:['Return', self.wpTakeoff],
                   5:['Land', self.local_home_position]}.get(self.step, 'END')

        if (process[0] == 'Takeoff') or(process[0] == 'WP1') or (process[0] == 'WP2') or (process[0] == 'WP3') or (process[0] == 'Return'):
            self.PubLocalPosition(process[1])

        elif (process[0] == 'Land'):
            self.setMode("AUTO.LAND")

        else:
            self.PublishWayPoint(self.step)
            rospy.loginfo('Mission Complete')
            rospy.signal_shutdown('Mission Complete')
            quit()

        self.PublishWayPoint(self.step)
        result = self.waypoint_reach_check(process, 0.5)

        if result is True:
            self.step += 1
            rospy.loginfo_once('Done')

        elif (process[0] == 'Land') and (self.current_state.armed is False):
            rospy.loginfo('Done')
            self.step += 1

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('main', anonymous=True, disable_signals=True)
    
    flight = Mission()

    rate = rospy.Rate(30.0)

    try:
        flight.check_FCU_connection()
        rospy.sleep(1)

        flight.check_home_setting()
        flight.setEarthRadius() 
        rospy.sleep(1)
        
        flight.setWayPoints()
        flight.setObstaclePoints()
        rospy.sleep(1)
        
        flight.setMode("OFFBOARD")
        rospy.sleep(1)

        flight.setArm()

        while not rospy.is_shutdown():
            flight.process()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
