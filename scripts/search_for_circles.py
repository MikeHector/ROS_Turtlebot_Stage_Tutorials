#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
import cv2
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
import cv_bridge
import numpy as np




class bot:
    def __init__(self):
        #Start node
        rospy.init_node('search_for_circles')
        #Start publishers
        self.image_pub = rospy.Publisher('scan_image', Image, queue_size=1)
        
        #Start subscribers        
        # self.image_sub = rospy.Subscriber("image_topic",Image,self.detect_circles)
        self.scan_sub = rospy.Subscriber("/scan",LaserScan, self.update_scan)
        self.amcl_pose = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.get_pose)
        #Initialize other properties
        self.bridge = cv_bridge.CvBridge() #Create bridge instance
        self.circle_locations = {}

    def update_scan(self,data):
        # print('getting data!')
        self.laser_scan_data = data
        # print('got data!')
        
    def build_image_from_scan(self):
        #Build mono8 image from scan data TODO: Make this better so the coordinate math is easier
        image_edge = 100 #Number of pixels around the edge of the image
        cm_per_pixel = 1
        range_scan_meters = 5 
        meters_to_pixels = 100.0 * (1.0 / cm_per_pixel)
        self.size = int(2*range_scan_meters*meters_to_pixels + 2*image_edge)
        image = np.ones((self.size,self.size),dtype=np.uint8)*255

        #Build the new image
        for i,ray in enumerate(self.laser_scan_data.ranges):
            angle = self.laser_scan_data.angle_increment * i + self.laser_scan_data.angle_min
            newCoordy = int(round(ray*meters_to_pixels*np.cos(angle),0)) + self.size/2
            newCoordx = int(round(ray*meters_to_pixels*np.sin(angle),0)) + self.size/2
            if ray < range_scan_meters:
                image[newCoordx,newCoordy] = 0
        return image
    
    def update_circle_loc(self,circle_loc):
        #Round to nearest .33
        circle_loc[0] = round(circle_loc[0]*3)/3
        circle_loc[1] = round(circle_loc[1]*3)/3
        c = (circle_loc[0], circle_loc[1])
        # print(self.circle_locations)
        if (c in self.circle_locations):
            self.circle_locations[c] += 1
            # print('1')
        else:
            self.circle_locations[c] = 1
            # print(self.circle_locations)
            # print('2')

    def get_pose(self, amcl_data):
        # self.pose = amcl_data
        D = amcl_data.pose.pose
        self.pose_x  = D.position.x
        self.pose_y =  D.position.y
        (a,b,self.pose_theta)= transformations.euler_from_quaternion((D.orientation.x, D.orientation.y, D.orientation.z, D.orientation.w))
        # print([a,b,self.theta])
        # self.pose_theta
    
    def circle_loc_to_world(self,bot_circle_loc_pixels):
        meter_per_pixel = .01
        p1x = (bot_circle_loc_pixels[0] - self.size/2) * meter_per_pixel
        p1y = (bot_circle_loc_pixels[1] - self.size/2) * meter_per_pixel
        # print([p1x, p1y])
        p1 = np.array([[p1x],
            [p1y],
            [1]])
        H01 = np.array([[np.cos(self.pose_theta), -np.sin(self.pose_theta),self.pose_x],
            [np.sin(self.pose_theta), np.cos(self.pose_theta), self.pose_y],
            [0,0,1]])
        M = np.matmul(H01,p1)
        # print([p1x, p1y, self.pose_x, self.pose_y, self.pose_theta], M[0,0],M[1,0])
        # print(M)
        return [M[0,0],M[1,0]]


    def detect_circles(self):
        #Declare a new image from laser_scan_data
        rospy.wait_for_message("/scan", LaserScan)
        rospy.wait_for_message("/amcl_pose",PoseWithCovarianceStamped) 
        image = self.build_image_from_scan()

        
        #Convert to rosImg and publish so that we can see it's working
        rosImg = self.bridge.cv2_to_imgmsg(image,'mono8')
        # self.image_pub.publish(rosImg)
        # to view image: rosrun image_view image_view image:=/scan_image
        
        #Create an opencv image from the rosImg message
        cvimage = self.bridge.imgmsg_to_cv2(rosImg,'mono8')
        #Run hough transform on it
        cimg = cv2.GaussianBlur(cvimage,(5,5),cv2.BORDER_DEFAULT)
        cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(cvimage,cv2.cv.CV_HOUGH_GRADIENT,2,100,
            param1=200,param2=14,minRadius=20,maxRadius=25)
        if circles is not None:
            #And trace around the circle
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle and innerdot
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                # print([i[0],i[1]])
                # print(self.circle_loc_to_world([i[0],i[1]]))

                circle_wp = self.circle_loc_to_world([i[0],i[1]])
                self.update_circle_loc(circle_wp)
        ros_stampedImg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
        ros_stampedImg = self.bridge.cv2_to_imgmsg(cimg,'bgr8')
        self.image_pub.publish(ros_stampedImg)

        # print(self.circle_locations)
        # self.set_markers()
        

    def image_from_scan(self,scan):
        pass

def merge_circle_dicts(d1,d2):
    if not d1:
        return d2
    if not d2:
        return d1
    else:
        new_d = d1.copy()
        for c in d2:
            if (c in d1):
                new_d[c] = d1[c]+d2[c]
            else:
                new_d[c] = d2[c]
        return new_d
        
def set_markers(circles,marker_pub):
        id = 1
        for location in circles:
            if circles[location] > 15:
                # print('1')
                marker = Marker()
                marker.id = id
                marker.header.frame_id = 'map'
                marker.type = Marker.SPHERE
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = location[0]
                marker.pose.position.y = location[1]
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0
                marker.scale.x = .2
                marker.scale.y = .2
                marker.scale.z = .2
                marker.lifetime.secs = 1000
                
                marker_pub.publish(marker)
                # print([c[0],c[1]])
            id += 1

if __name__ == '__main__':
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    circles = {}
    go = True
    while go:
        n = bot()
        try:
            n.detect_circles()
            circles = merge_circle_dicts(circles, n.circle_locations)
            print(circles)
            set_markers(circles,marker_pub)
        except rospy.ROSInterruptException:
            go = False
