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

class PersonTracker:
	def __init__(self, max_size=1.5, min_size=0.01, axis_a=0.9, center_a=0.1):
        self.max_size = max_size
        self.min_size = min_size
        self.axis_alpha = axis_a
        self.center_alpha = center_a
        self.last_a = None
        self.last_b = None
        self.last_center = None

        self.centers = []
        self.filter_size = 10

        self.scan_frame_id = "laser"

        self.person_marker_pub = rospy.Publisher("persons_marker", Marker, queue_size=10)
        self.filtered_sub = rospy.Subscriber("filtered_scan", LaserScan, self.find_person_from_scan, self.person_marker_pub)

    def median_filter(self, data):
        self.centers.append(data)
        if len(self.centers) > self.filter_size:
            self.centers.pop(0)

        x = []
        y = []
        for item in self.centers:
            x.append(item[0].real)
            y.append(item[1].real)

        return [np.median(x), np.median(y)]

    def find_person_from_scan(self, data, pub):

    	ellipse_xy = []
    	points = []

    	angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges

        for r in ranges:
        	points.append([cos(angle)*r, sin(angle)*r])

        	angle += incr
