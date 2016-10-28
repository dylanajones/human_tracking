#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import *
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan

from math import sin,cos,atan2,pi,sqrt
import matplotlib.pyplot as plt

from ellipse2d import Ellipse2d
from sklearn.cluster import DBSCAN

# Class which handles tracking and contains the subscriber and publisher

class HumanTracker:
    # Initialize with the topic to subscribe to and 
    def __init__(self, scan_topic, max_size=1.0, min_size=0.01 ):

        self.max_size = max_size
        self.min_size = min_size

        self.scan_topic = scan_topic

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.find_in_scan)

        # File output for testing
        # self.file_name = "../data/DBSCAN_test.txt"
        # self.file_target = open(self.file_name, 'w')
        

    def shutdown(self):
        self.file_target.close()

    # Function called on recieving a laser scan
    def find_in_scan(self, data):

        # Convert scan into x-y points
        scan_xy = []

        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges

        for r in ranges:
            if r < max_range:
                scan_xy.append([cos(angle)*r, sin(angle)*r])
            angle += incr

        # Using DBSCAN Algorithm to cluster points
        # eps tells size of the neigberhood and min tells how many samples
        # need to be there for cluster to count
        db = DBSCAN(eps = 1.5, min_samples = 5).fit(scan_xy)
        labels = db.labels_

        #print(labels)
        # Number of clusters - removing -1 because that labels noise
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        # Grouping points into lists
        groups = [[] for ii in range(n_clusters)]

        for ii, item in enumerate(labels):
            # Label -1 represents noise in DBSCAN algorithm
            if item != -1:
                groups[item].append(scan_xy[ii])


        people_count = 0

        colors = plt.cm.Spectral(np.linspace(0, 1, n_clusters))

        # Go through all groups and see if the are humans
        for ii, item in enumerate(groups):
            if len(item) > 0:

                try:
                    # Fitting ellipse to the point set - Code from ellipse2d.py
                    ellipse = Ellipse2d()
                    ellipse.fit(item)

                    # Checking size and location of ellipse - want roughly human sized
                    if self.is_valid_person_ellipse(ellipse, self.max_size, self.min_size):
                        if not(-0.5 < sqrt(ellipse.center[0] ** 2 + ellipse.center[1] ** 2) < 0.5):
                            # Do here what we want when a human is found
                            # TODO:
                            #   - Write test for distance of center
                            #   - Write code to tell system to stop if human too close

                            people_count += 1
                            print "Finding Human: ", people_count
                            self.file_target.write(str(ellipse.center[0].real))
                            self.file_target.write(",")
                            self.file_target.write(str(ellipse.center[1].real))
                            self.file_target.write(",")
                            self.file_target.write(str(people_count))
                            self.file_target.write("\n")
                            #plt.plot(ellipse.center[0],ellipse.center[1],'r+')
                            #x,y = apply(zip,item)
                            #plt.scatter(x,y,c=colors[ii])
                            #print ellipse                  
                except:
                    pass



        #Code for plotting / testing ...
        # print(n_clusters)

        # colors = plt.cm.Spectral(np.linspace(0, 1, n_clusters))

        # for ii, item in enumerate(groups):
        #     x,y = apply(zip,item)
        #     plt.scatter(x,y,c=colors[ii])

        # plt.show()
        #End code for plotting and testing

    def is_valid_person_ellipse(self, ellipse, max_size=1.0, min_size=0.01):
        # Validity is measured by it being a real ellipse, with 
        #   values in the plausible range for representing a human

        if (ellipse.is_valid() and 
            (min_size < ellipse.a < max_size) and 
            (min_size < ellipse.b < max_size)):
            return True
        else:
            return False




if __name__ == '__main__':

    rospy.init_node("human_tracker", anonymous = False)

    human_tracker = HumanTracker("scan")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        human_tracker.shutdown()