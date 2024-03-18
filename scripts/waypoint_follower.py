#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np

class Controller:
    def __init__(self):
        self.state = State()
        self.current_pose = Point()
        self.home_position = [0, 0, 0]
        self.waypoints = rospy.get_param('waypoints')
        self.wp_index = 0
        
        # Publisher
        self.local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCb)
        
        # Service_client
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        
        # Timer
        main_timer = rospy.Timer(rospy.Duration(0.1), self.process)
    
    def checkFcuConnection(self):
        while not self.state.connected:
            rospy.logwarn_throttle(1, "Wait FCU connection")

        rospy.logwarn("FCU connected")

    def setMode(self, mode):
        rate = rospy.Rate(0.5)
        while True:
            self.publishGoalPose(self.home_position)

            if self.state.mode != mode:
                self.set_mode_client(base_mode=0, custom_mode=mode)
                rospy.logwarn("Setting to OFFBOARD mode...")
            else:
                break

            rate.sleep()
                
    def setArm(self):
        rate = rospy.Rate(0.5)
        while True:
            if self.state.armed is not True:
                self.arming_client(True)
            else:
                break

            rate.sleep()

    def stateCb(self, msg):
        prev_state = self.state
        self.state = msg
        
        if self.state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.state.mode)

        if self.state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.state.armed)
        
    def poseCb(self, msg):
        self.current_pose.x = msg.pose.position.x
        self.current_pose.y = msg.pose.position.y
        self.current_pose.z = msg.pose.position.z

    def publishGoalPose(self, point):
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]

        self.local_pose_pub.publish(pose)

    def isGoalReached(self, wp, reach_range):
        x = self.current_pose.x
        y = self.current_pose.y
        z = self.current_pose.z

        d = np.sqrt((wp[0] - x)**2 + (wp[1] - y)**2 + (wp[2] - z)**2)

        if d < reach_range :
            return True
        else:
            rospy.loginfo_throttle(1, 'Process <Distance: %.1f>'%(d))
            return False


    def process(self, event):
        if self.state.mode == "OFFBOARD":
            goalPoint = self.waypoints[self.wp_index]
            
            if(self.isGoalReached(goalPoint, 3.0)):
                self.wp_index = min(self.wp_index + 1, len(self.waypoints) - 1)
                
                if self.wp_index == len(self.waypoints) - 1:
                    rospy.loginfo('DONE!')
                else:
                    rospy.loginfo('Waypoint updated!')
            goalPoint = self.waypoints[self.wp_index]
            
            rospy.loginfo_throttle(3, "Current : X: %.2f  Y: %.2f  Z: %.2f"%(self.current_pose.x, self.current_pose.y, self.current_pose.z))
            rospy.loginfo_throttle(3, "Target : X: %.2f  Y: %.2f  Z: %.2f"%(goalPoint[0], goalPoint[1], goalPoint[2]))

            self.publishGoalPose(goalPoint)
            


if __name__ == '__main__':
    rospy.init_node('waypoint_follower_node', anonymous=True, disable_signals=True)
    
    ctr = Controller()
    rate = rospy.Rate(30.0)

    try:
        ctr.checkFcuConnection()
        rospy.sleep(1)
        
        ctr.setMode("OFFBOARD")
        rospy.sleep(1)

        ctr.setArm()

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass