#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from numpy import linspace, sqrt, arctan2


class bot:
    def __init__(self):
        #Initialize Node
        rospy.init_node('move', anonymous=True)
        #Start Publishes
        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        #Start Subscribers
        self.pose_sub = rospy.Subscriber("/scan", LaserScan, self.update_scan)
        #Set any ROS parameters     
        rospy.set_param('stop_distance',0.75)
        #Set any other class properties 
        # self.min_range = 50  
        #Limit update rate
        self.rate = rospy.Rate(10)

        #Sleep so that topics get published
        rospy.sleep(.001)

    def update_scan(self, data):
        range = list(data.ranges)
        #angle = linspace(data.angle_min,data.angle_max,len(range))

        self.distance_to_wall = min(range)

    def update_pose(self, data):
        self.x = round(data.x,4)
        self.y = round(data.y,4)
        self.ang = data.theta
        self.d_lin = data.linear_velocity
        self.d_ang = data.angular_velocity

    def distance_to_goal(self, goal):
        return sqrt((goal.x - self.x)**2 + (goal.y - self.y)**2)

    def linear_velocity(self, goal, linear_gain = 1.5):
        return linear_gain * self.distance_to_goal(goal)

    def angle_to_goal(self, goal):
        return arctan2((goal.y-self.y),(goal.x-self.x))

    def angular_velocity(self, goal, angular_gain = 7):
        return angular_gain * (self.angle_to_goal(goal) - self.ang)

    def get_inputs(self):
        goal_pose = Pose()
        goal_pose.x = input("x goal: ")
        goal_pose.y = input("y goal: ")
        tolerance = input("Acceptable error from goal ")
        assert ((goal_pose.x >= 0) and (goal_pose.y >= 0) and (goal_pose.x <= 10) and (goal_pose.y <= 10)), \
             "Goal poses must be between 0 and 10"

        return goal_pose, tolerance

    def go_to_pos(self):
        goal = Pose()

        #Poll for goals and tolerance
        #To do: implement limits on these
        goal, dist_tolerance = self.get_inputs()
            
        #Initialize twist struct
        vel_msg = Twist()

        while self.distance_to_goal(goal) >= dist_tolerance:
            vel_msg.linear.x = self.linear_velocity(goal)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_velocity(goal)

            #Publish vel msg and rate
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        #Stop
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        print('Made it!')

    def drive_fwd(self):
        vel_msg = Twist()

        fwd_vel = .1

        vel_msg.linear.x = fwd_vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        while True:
            print([self.distance_to_wall,rospy.get_param('stop_distance')])
            if self.distance_to_wall < rospy.get_param('stop_distance'):
                vel_msg.linear.x = 0
                print('stopped!')
            else:
                vel_msg.linear.x = fwd_vel

            self.vel_pub.publish(vel_msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    try:
        n = bot()
        n.drive_fwd()
    except rospy.ROSInterruptException:
        pass
