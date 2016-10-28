#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import *
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from copy import deepcopy
from munkres import Munkres
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from math import sin,cos,atan2,pi,sqrt

from scipy.optimize import linear_sum_assignment

class WallFilter:
    
    def __init__ (self, scan_topic):

        self.filter_scans = []
        self.filter_size = 10

        self.max_var = 0.1

        self.scan_topic = scan_topic

        # Make these update from odom messages
        self.odom_x = 0
        self.odom_y = 0

        self.filter_scan_pub = rospy.Publisher("filtered_scan", PointCloud2, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.findMovement, self.filter_scan_pub)

    def findMovement(self, data, publisher):
        m = Munkres()

        print "processing new scan"

        convert_data = self.xyConvert(data)

        if not(len(self.filter_scans) < self.filter_size):
            print "starting filter"
            self.update_filter(convert_data)

            print "adding odom"
            odom_scan = self.addOdom()

            matches = [[] for ii in range(len(self.filter_scans[1]))]

            for ii in range(1,self.filter_size - 1):
                print "calculating cost mat"
                cost_mat = self.makeCostMatrix(self.filter_scans[0], self.filter_scans[ii])
                print len(cost_mat)
                print "doing matching"
                #indexes = m.compute(cost_mat)
                row_ind, col_ind = linear_sum_assignment(cost_mat)

                print "done matching for scan: ", ii
                #for item in indexes:
                 #   matches[item[0]].append(self.filter_scans[ii][item[1]])

                for jj, item in enumerate(row_ind):
                    matches[item].append(self.filter_scans[ii][jj])

            matched_scans = []

            print "processing"
            for item in matches:
                res = apply(zip, item)
                matched_scans.append(res)

            filtered_points = []

            print "computing avg and std"
            for item in matched_scans:
                x_avg = np.mean(item[0])
                x_std = np.std(item[0])
                y_avg = np.mean(item[1])
                y_std = np.std(item[1])

                if x_std > self.max_var and y_std > self.max_var:
                    filtered_points.append([x_avg, y_avg, 0])

            print filtered_points
            pc_cloud = PointCloud2()
            pc_cloud = pc2.create_cloud_xyz32(pc_cloud.header, filtered_points)
            publisher.publish(pc_cloud)
            print "ending filter"

        else:
            print "building filter"
            self.filter_scans.append(convert_data)

    def xyConvert(self, data):
        points = []

        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges

        for r in ranges:
            points.append([cos(angle)*r, sin(angle)*r])

            angle += incr

        return points

    def update_filter(self, data):
        # Remove last thing in filter
        self.filter_scans.pop(0)
        # Add new thing
        self.filter_scans.append(data)

    def addOdom(self):

        odom_altered_scans = deepcopy(self.filter_scans)

        for ii, item in enumerate(odom_altered_scans):
            if ii != 0:
                for point in item:
                    point[0] += self.odom_x
                    point[1] += self.odom_y

        return odom_altered_scans

    def makeCostMatrix(self, vec_1, vec_2):

        #print vec_1
        #print vec_2

        cost_mat = []

        for ii, item_1 in enumerate(vec_1):
            cost_mat.append([])
            for jj, item_2 in enumerate(vec_2):
                cost_mat[ii].append(self.dist(item_1, item_2))

        return cost_mat

    def dist(self, item_1, item_2):

        dif_sum = 0

        #print item_1, "item 1"
        #print item_2, "item 2"

        for ii, item in enumerate(item_1):
            dif_sum += (item - item_2[ii]) ** 2

        #print dif_sum, "dif sum"
        return sqrt(dif_sum)

if __name__ == '__main__':
    
    rospy.init_node("wall_filter", anonymous=False)

    # scan_topic = rospy.get_param("~scan_topic")
    
    wall_filter = WallFilter("scan")
    
    rospy.spin()