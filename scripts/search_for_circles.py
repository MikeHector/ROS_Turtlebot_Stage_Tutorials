#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
import cv2
from sensor_msgs.msg import Image, LaserScan
import cv_bridge
import numpy as np




class bot:
    def __init__(self):
        #Start node
        rospy.init_node('search_for_circles')
        #Start publishers
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.image_pub = rospy.Publisher('scan_image', Image, queue_size=1)
        
        #Start subscribers        
        # self.image_sub = rospy.Subscriber("image_topic",Image,self.detect_circles)
        self.scan_sub = rospy.Subscriber("/scan",LaserScan, self.update_scan)
        #Initialize other properties
        self.circle_locations = []
        # rospy.sleep(.01)
        
        

    def update_Marker(self,wp): 
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

    def update_scan(self,data):
        self.laser_scan_data = data
        
    
    def detect_circles(self):
        #Declare a new image and build it out of the laser_scan_data
        image_edge = 50 #Number of pixels around the edge of the image
        cm_per_pixel = 1
        range_scan_meters = 5 
        meters_to_pixels = 100.0 * (1.0 / cm_per_pixel)
        size = int(range_scan_meters*meters_to_pixels + image_edge)
        image = np.ones((size,size),dtype=np.uint8)*255
        
        # print(self.laser_scan_data)
        
        #Build the new image
        for i,ray in enumerate(self.laser_scan_data.ranges):
            angle = self.laser_scan_data.angle_increment * i
            newCoordx = int(round(ray*meters_to_pixels*np.cos(angle),0))
            newCoordy = int(round(ray*meters_to_pixels*np.sin(angle),0))
            image[newCoordx,newCoordy] = 0
        
        cvb_inst = cv_bridge.CvBridge() #Create instance to add image to it
        
        #Convert to rosImg and publish so that we can see it's working
        rosImg = cvb_inst.cv2_to_imgmsg(image,'mono8')
        self.image_pub.publish(rosImg)
        # to view image: rosrun image_view image_view image:=/scan_image
        
        #Create a cv image from the rosImg message
        cvimage = cvb_inst.imgmsg_to_cv2(rosImg,'mono8')
        #Run hough transform on it
        # cvimage = cv2.medianBlur(cvimage,5)
        cimg = cv2.cvtColor(cvimage,cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(cvimage,cv2.cv.CV_HOUGH_GRADIENT,1,200,
            param1=1,param2=5,minRadius=25,maxRadius=30)
        if circles is not None:
            #And trace around the circle
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                print(i,' circle!', i[0], i[1])
            # cv2.imshow('circles',cimg)
        ros_stampedImg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
        ros_stampedImg = cvb_inst.cv2_to_imgmsg(cimg,'bgr8')
        self.image_pub.publish(ros_stampedImg)

    def image_from_scan(self,scan):
        pass


        
if __name__ == '__main__':
    while True:
        try:
            n = bot()
            rospy.sleep(.2)
            n.detect_circles()
        except rospy.ROSInterruptException:
            pass
