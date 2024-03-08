#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Path


class Visualizer:
    def __init__(self):
        self.current_pose = PoseStamped()
        self.waypoints = rospy.get_param('waypoints')
        self.fcuPath = Path()
        
        self.NUM_MESHES = 11
        self.NUM_PINETREE = 10
        self.meshFiles = [
            "package://chatgpt_gazebo/models/pine_tree/meshes/pine_tree.dae",
            "package://chatgpt_gazebo/models/house_1/meshes/house_1.dae"
        ]
        
        self.modelPoses = [
            [-2.4, 0.102443, 0.0],  # pine_tree
            [-8.0, 10.0, 0.0],      # pine_tree_8
            [4.0, 7.0, 0.0],        # pine_tree_0
            [-4.0, 15.0, 0.0],      # pine_tree_1
            [11, -1.5, 0.0],        # pine_tree_2
            [5.0, 17.0, 0.0],       # pine_tree_3
            [13.0, 10.0, 0.0],      # pine_tree_4
            [-5.0, 25.0, 0.0],      # pine_tree_5
            [19.0, 2.0, 0.0],       # pine_tree_6
            [18.0, 17.0, 0.0],      # pine_tree_7
            [10.9831, 30.7394, 0.0],# house_1
        ]
        
        self.fcuPoseSub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCb)
        self.pathPub = rospy.Publisher('fcuPath', Path, queue_size=10)
        self.waypointPub = rospy.Publisher('waypoint_marker', MarkerArray, queue_size=10)
        self.markerPub = rospy.Publisher('model_meshes', MarkerArray, queue_size=10)
        
    def poseCb(self, msg):
        self.current_pose = msg
        
        self.fcuPath.header.stamp = msg.header.stamp
        self.fcuPath.header.frame_id = 'map'
        self.fcuPath.poses.append(msg)
        self.pathPub.publish(self.fcuPath)
        
    def publishGoalMarker(self):
        waypoints = MarkerArray()
        
        for i, wp in enumerate(self.waypoints):
            waypoint = Marker()
            waypoint.header.frame_id = "map"
            waypoint.header.stamp = rospy.Time.now()
            waypoint.ns = "waypoint"
            waypoint.id = i
            waypoint.type = Marker.SPHERE
            waypoint.action = Marker.ADD
            waypoint.pose.position.x = wp[0]
            waypoint.pose.position.y = wp[1]
            waypoint.pose.position.z = wp[2]
            waypoint.pose.orientation.x = 0
            waypoint.pose.orientation.y = 0
            waypoint.pose.orientation.z = 0
            waypoint.pose.orientation.w = 1
            waypoint.scale.x = 1.0
            waypoint.scale.y = 1.0
            waypoint.scale.z = 1.0
            waypoint.color.a = 1.0
            waypoint.color.r = 1.0
            waypoint.color.g = 0.0
            waypoint.color.b = 0.0
            waypoints.markers.append(waypoint)
            
        self.waypointPub.publish(waypoints)

    def publishMeshMarker(self):
        meshes = MarkerArray()
        for i in range(self.NUM_MESHES):
            marker = Marker()
            if i < self.NUM_PINETREE:
                marker.mesh_resource = self.meshFiles[0]
            else:
                marker.mesh_resource = self.meshFiles[1]
            marker.header.frame_id = "map" 
            marker.header.stamp = rospy.Time.now()
            
            marker.ns = "mesh"
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.pose.position.x = self.modelPoses[i][0]
            marker.pose.position.y = self.modelPoses[i][1]
            marker.pose.position.z = self.modelPoses[i][2]
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            meshes.markers.append(marker)
        self.markerPub.publish(meshes)
        
      

if __name__ == '__main__':
    rospy.init_node('visualizer_node', anonymous=True)
    vis = Visualizer()
    rate = rospy.Rate(30.0)
    
    try:
        while not rospy.is_shutdown():
            vis.publishGoalMarker()
            vis.publishMeshMarker()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
