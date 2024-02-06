#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

## TODO: Specify the name of objects in Gazebo world
# objects_dict = {
#     "turbine1": "BP_Wind_Turbines_C_1",
#     "turbine2": "StaticMeshActor_2",
#     "solarpanels": "StaticMeshActor_146",
#     "crowd": "StaticMeshActor_6",
#     "car": "StaticMeshActor_10",
#     "tower1": "SM_Electric_trellis_179",
#     "tower2": "SM_Electric_trellis_7",
#     "tower3": "SM_Electric_trellis_8",
# }

def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    roll is rotation around x-axis, pitch is rotation around y-axis
    and yaw is rotation around z-axis.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z

def calc_xy_err(cur, dest):
    xy_err = math.sqrt((cur.pose.position.x - dest.pose.position.x)**2 + (cur.pose.position.y - dest.pose.position.y)**2)
    return xy_err

def calc_z_err(cur, dest):
    z_err = math.sqrt((cur.pose.position.z - dest.pose.position.z)**2)
    return z_err
    
class MavrosWrapper:
    def __init__(self):
        self.state = State()
        self.target_pose = PoseStamped()
        
        self.rate = rospy.Rate(20) # 20 Hz (setpoint publish should be faster than 2Hz)
        self.service_timeout = 30
        
        self.target_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        try:
            rospy.wait_for_service('/mavros/cmd/arming', self.service_timeout)
            rospy.wait_for_service('/mavros/set_mode', self.service_timeout)
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        while not self.state.connected:
            rospy.loginfo_throttle(1, "Wait FCU Connection")
        rospy.loginof("FCU Connected")
        
        while True:
            if self.state.armed is not True:
                self.arming_client(True)
            else:
                break
            self.rate.sleep()
      
    def state_callback(self, msg):
        self.state = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def set_mode(self, mode):
        try:
            response = self.set_mode_client(base_mode = 0, custom_mode = mode)
            if response.mode_sent:
                rospy.loginfo("Mode Changed to %s" % mode)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def takeoff(self):
        while abs(self.target_pose.pose.position.z - self.current_pose.pose.position.z) > 0.1:
            self.target_pose = PoseStamped()
            self.target_pose.pose.position.x = self.current_pose.pose.position.x
            self.target_pose.pose.position.y = self.current_pose.pose.position.y
            self.target_pose.pose.position.z = 3.0
            self.target_pose.pose.orientation.x = self.current_pose.pose.orientation.x
            self.target_pose.pose.orientation.y = self.current_pose.pose.orientation.y
            self.target_pose.pose.orientation.z = self.current_pose.pose.orientation.z
            self.target_pose.pose.orientation.w = self.current_pose.pose.orientation.w
            self.target_pose_pub(self.target_pose)
            # Setpoint should be streamed before entering OFFBOARD
            if self.state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
            self.rate.sleep() # 20 Hz
        

    def land(self):
        try:
            response = self.set_mode_client(base_mode = 0, custom_mode = "AUTO.LAND")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed : %s" %e)
        rospy.loginfo("Landing...")
        if response.mode_sent:
            # wait until the drone is disarmed
            while self.current_state.armed:
                rospy.loginfo("Disarming...")
                self.arming_client(False)
                rospy.sleep(1)
        rospy.loginfo("Landed")

    def get_drone_position(self):
        pose = self.current_pose
        return [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

    def fly_to(self, point):
        self.target_pose.pose.position.x = point[0]
        self.target_pose.pose.position.y = point[1]
        self.target_pose.pose.position.z = point[2]
        self.target_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        self.target_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        self.target_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        self.target_pose.pose.orientation.w = self.current_pose.pose.orientation.w
        while calc_xy_err(self.target_pose, self.current_pose) > 0.3 or calc_z_err(self.target_pose, self.current_pose) > 0.2:
            self.target_pose_pub(self.target_pose)

    def fly_path(self, points):
        for point in points:
            self.fly_to(point)
            
    def set_yaw(self, yaw):
        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = self.current_pose.pose.position.z
        w, x, y, z= euler_to_quaternion(0.0, 0.0, yaw)
        self.target_pose.pose.orientation.x = x
        self.target_pose.pose.orientation.y = y
        self.target_pose.pose.orientation.z = z
        self.target_pose.pose.orientation.w = w
        self.target_pose_pub(self.target_pose)

    def get_yaw(self):
        x = self.current_pose.pose.orientation.x
        y = self.current_pose.pose.orientation.y
        z = self.current_pose.pose.orientation.z
        w = self.current_pose.pose.orientation.w
        yaw = quaternion_to_euler(w, x, y, z)
        return yaw

## TODO Get the position of gazebo model with '/gazebo/model_state'
    # def get_position(self, object_name):
    #     query_string = objects_dict[object_name] + ".*"
    #     object_names_ue = []
    #     while len(object_names_ue) == 0:
    #         object_names_ue = self.client.simListSceneObjects(query_string)
    #     pose = self.client.simGetObjectPose(object_names_ue[0])
    #     return [pose.position.x_val, pose.position.y_val, pose.position.z_val]