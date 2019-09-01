#!/usr/bin/env python
import rospy
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def waypoint_to_goal_pose(waypoint):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = waypoint[0][0]
    goal_pose.target_pose.pose.position.y = waypoint[0][1]
    goal_pose.target_pose.pose.position.z = waypoint[0][2]
    goal_pose.target_pose.pose.orientation.x = waypoint[1][0]
    goal_pose.target_pose.pose.orientation.y = waypoint[1][1]
    goal_pose.target_pose.pose.orientation.z = waypoint[1][2]
    goal_pose.target_pose.pose.orientation.w = waypoint[1][3]
    return goal_pose


class bot:
    def __init__(self):
        #Start node
        rospy.init_node('nav_wp')
        #Start publishers
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        #Start movebase client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.waypoints = [
            [(2.1, 2.2, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(6.5, 4.43, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)]
        ]
    def update_Marker(self,wp,goal): #TODO: this function only needs goal; pull the position info out of it instead of wp
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.type = Marker.SPHERE
        self.marker.color.r = 0
        self.marker.color.g = 1
        self.marker.color.b = 0
        self.marker.color.a = 1
        self.marker.pose.position.x = wp[0][0]
        self.marker.pose.position.y = wp[0][1]
        self.marker.pose.position.z = wp[0][2]
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = .2
        self.marker.scale.y = .2
        self.marker.scale.z = .2
        self.marker.lifetime.secs = 10

        self.marker_pub.publish(self.marker)

    
    def nav(self):
        for wp in self.waypoints:
            goal = waypoint_to_goal_pose(wp)
            self.update_Marker(wp,goal)
            self.client.send_goal(goal)
            self.client.wait_for_result()


        
if __name__ == '__main__':
    while True:
        try:
            n = bot()
            n.nav()
        except rospy.ROSInterruptException:
            pass
