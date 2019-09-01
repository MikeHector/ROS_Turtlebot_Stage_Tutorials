#!/usr/bin/env python
import rospy
import actionlib
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
        #Start movebase client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.waypoints = [
            [(2.1, 2.2, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(6.5, 4.43, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)]
        ]
    
    def nav(self):
        for wp in self.waypoints:
            goal = waypoint_to_goal_pose(wp)
            self.client.send_goal(goal)
            self.client.wait_for_result()


        
if __name__ == '__main__':
    while True:
        try:
            n = bot()
            n.nav()
        except rospy.ROSInterruptException:
            pass
