#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np

from math import sin,cos,atan2,pi,sqrt
import matplotlib.pyplot as plt

from ellipse2d import Ellipse2d
#from human_tracker.msg import PersonLocation2D

class PersonTracker:
    def __init__(self, max_size=1.5, min_size=0.01, axis_a=0.9, center_a=0.1):
        self.max_size = max_size
        self.min_size = min_size
        self.axis_alpha = axis_a
        self.center_alpha = center_a
        self.last_a = None
        self.last_b = None
        self.last_center = None
        # self.last_x
        # self.last_y

        self.centers = []
        self.filter_size = 10

        #self.file_name = "../data/ellipse_center_headlands10ft_small_wallfilter.txt"
        self.file_name = "../data/ellipse_center_headlands10ft_updatingwallfilter.txt"
        self.file_target = open(self.file_name, 'w')

        # self.red = ColorRGBA(1, 0, 0, 1)
        # self.green = ColorRGBA(0, 1, 0, 1)
        # self.color = self.red

        self.scan_frame_id = "laser"

        self.person_marker_pub = rospy.Publisher("persons_marker", Marker, queue_size=10)
        #self.person_location_pub = rospy.Publisher("persons_location", PersonLocation2D, queue_size=10)
        self.filtered_sub = rospy.Subscriber("filtered_scan", LaserScan, self.find_person_from_scan, self.person_marker_pub)
        #self.updated_filter_cmd_sub = rospy.Subscriber("update_filter_cmd", Bool, self.reset)
        #self.contam_colors_sub = rospy.Subscriber("contam", Float32, self.get_colors)

    def shutdown(self):
        self.file_target.close()

    def reset(self, run):
        self.last_a = None
        self.last_b = None
        self.last_center = None
        self.color = self.red

    def get_colors(self, data):
        if data.data < 0.5:
            self.color = self.red
        else:
            self.color = self.green

    def median_filter(self, data):
        #print "HERE!!!!"
        self.centers.append(data)
        if len(self.centers) > self.filter_size:
            self.centers.pop(0)

        x = []
        y = []
        for item in self.centers:
            x.append(item[0].real)
            y.append(item[1].real)

        return [np.median(x), np.median(y)]


            


    #turn filtered laser data into markers
    def find_person_from_scan(self, data, pub):
        #print "fitting"
        ellipse_xy = []
        #points = [] #array to hold all points - FOR DEBUGGING

        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges
        #polar >> cartesian

        for r in ranges:
            if r < max_range:
                ellipse_xy.append([cos(angle)*r, sin(angle)*r]) #make xy
            angle += incr

        if len(ellipse_xy) > 0:
            #eliminate outlying points
            x_avg = sum([xy[0] for xy in ellipse_xy])/len(ellipse_xy)
            y_avg = sum([xy[1] for xy in ellipse_xy])/len(ellipse_xy)
            for xy in ellipse_xy:
                if (abs(xy[0]-x_avg)<0.5 or abs(xy[1]-y_avg)<0.5):
                    ellipse_xy.remove(xy)
            #fit ellipse to points - constrain size

        if len(ellipse_xy) > 1:
            #print "getting here"
            #print (ellipse_xy)
            try:
                ellipse = Ellipse2d()
                ellipse.fit(ellipse_xy)

                if self.is_valid_person_ellipse(ellipse, self.max_size, self.min_size): 
                    #apply alpha to smooth changes over time, if old data exists
                    #print "finding person"
                    if self.last_a != None and self.last_b != None and self.last_center != None:
                        ellipse.center = [self.last_center[i]*self.center_alpha + ellipse.center[i]*(1-self.center_alpha) for i in [0, 1]]
                        ellipse.a = self.last_a*self.axis_alpha + ellipse.a*(1-self.axis_alpha)
                        ellipse.b = self.last_b*self.axis_alpha + ellipse.b*(1-self.axis_alpha)
                    
                    self.last_center = self.median_filter(ellipse.center)
                    self.last_b = ellipse.b
                    self.last_a = ellipse.a

                    # publish the marker associated with the person
                    marker = self.create_person_marker(self.last_center[0], self.last_center[1], ellipse.theta, ellipse.a, ellipse.b)
                    self.person_marker_pub.publish(marker)

                    #print "here"
                    #print self.last_center 

                    #print "writing to file"
                    # Write locations to file
                    # self.file_target.write("TESTING, ")
                    self.file_target.write(str(self.last_center[0]))
                    self.file_target.write(",")
                    self.file_target.write(str(self.last_center[1]))
                    self.file_target.write("\n")

                    # publish the location of the people in the frame of the map
                    # person = self.create_person_data(ellipse.center[0], ellipse.center[1], ellipse.theta, ellipse.a, ellipse.b)
                    # pub.publish(person)
                #else:

                	#print "Not finding person"
            except:
                pass
            
        
    def is_valid_person_ellipse(self, ellipse, max_size=1.0, min_size=0.01):
        # Validity is measured by it being a real ellipse, with 
        #   values in the plausible range for representing a human

        #print ellipse.a
        #print ellipse.b

        if (ellipse.is_valid() and 
            (min_size < ellipse.a < max_size) and 
            (min_size < ellipse.b < max_size)):
            return True
        else:
            return False

    # def create_person_data(self, pose_x, pose_y, pose_theta, ellipse_a, ellipse_b, name="unknown"):
    #     h = Header()
    #     h.frame_id = self.scan_frame_id
    #     h.stamp = rospy.Time.now()

    #     person = PersonLocation2D()
    #     person.header = h
    #     person.name = name
    #     person.pose.x = pose_x
    #     person.pose.y = pose_y
    #     person.pose.theta = pose_theta
    #     person.ellipse_a = ellipse_a
    #     person.ellipse_b = ellipse_b
    #     person.contamination = 0.05

    #     return person

    def create_person_marker(self, pose_x, pose_y, ellipse_theta, ellipse_a, ellipse_b):
        h = Header()
        h.frame_id = self.scan_frame_id #tie marker visualization to laser it comes from
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "person_marker"
        mark.id = 0
        mark.type = 3
        mark.action = 0
        mark.scale = Vector3(ellipse_a*2, ellipse_b*2, 1) #scale, in meters
        mark.color = ColorRGBA(0, 1, 0, 1) #marker is red if clean, green if infected

        pose = Pose(Point(pose_x, pose_y, 0.5), Quaternion(0.0,0.0,1.0,cos(ellipse_theta/2)))
        mark.pose = pose

        return mark


if __name__ == '__main__':
    rospy.init_node("person_tracker", anonymous=False)

    # max_size = rospy.get_param("~max_size")
    # min_size = rospy.get_param("~min_size")
    # axis_alpha = rospy.get_param("~axis_alpha")
    # axis_center = rospy.get_param("~axis_center")

    # tracker = PersonTracker(max_size, min_size, axis_alpha, axis_center)

    tracker = PersonTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        tracker.shutdown()