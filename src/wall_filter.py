#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import *
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan

class WallFilter:
    def __init__ (self, scan_topic):
        self.filter_set = False
        #note wall offsets slightly between each scan so two arrays are needed
        self.walls = [[],[]]
        self.new_walls = [[]]
        self.reset_count = 0
        self.reset_thresh = 10
        self.switch = False

        self.scan_topic = scan_topic

        self.filter_scan_pub = rospy.Publisher("filtered_scan", LaserScan, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._rm_walls, self.filter_scan_pub) #needs to be subscribed to wall laser topic
        # self.filter_cmd_sub = rospy.Subscriber("update_filter_cmd", Bool, self.reset_filter)

    #When the room is empty call filter reset
    def reset_filter(self, run):
        self.filter_set = False

    def _rm_walls(self, data, publisher):
        variance = 0.3 #to account for noise
        switch = int(self.switch)
        self.switch = not self.switch #toggle switch
        
        #if the filter is in place remove walls
        if self.filter_set:
            walls = self.walls[switch]
            ranges = data.ranges
            filtered_ranges = []
            
            for i in xrange(len(walls)):
                try:
                    if ranges[i] < (walls[i]-variance):
                        filtered_ranges.append(ranges[i])
                    else:
                        filtered_ranges.append(data.range_max+1) #invalidate the result at this point
                except IndexError:
                    filtered_ranges.append(data.range_max+1)
            
            #publish filtered_ranges as LaserScan
            filtered_scan = data
            h = std_msgs.msg.Header()
            h.stamp = data.header.stamp
            h.frame_id = data.header.frame_id
            filtered_scan.header = h
            filtered_scan.ranges = filtered_ranges
            publisher.publish(filtered_scan)
        
        #if the filter reset has been called use data to change filter instead
        else:
            #scan the room a specified # of times into an array
            if self.reset_count < self.reset_thresh:
                self.new_walls.append([])
                self.new_walls[self.reset_count] = data.ranges
                self.reset_count += 1
            
            elif self.reset_count == self.reset_thresh:
                #unzip new_walls (to go by point instead of dataset) - len should be ~180
                zipped0 = map(list, zip(*filter(None, self.new_walls[0::2])))
                zipped1 = map(list, zip(*filter(None, self.new_walls[1::2])))
                self.walls[0] = [numpy.median(z) for z in zipped0]
                self.walls[1] = [numpy.median(z) for z in zipped1]
                
                #reset vars
                self.reset_count = 0
                self.new_walls = [[]]
                self.filter_set = True
            

if __name__ == '__main__':
    
    rospy.init_node("wall_filter", anonymous=False)

    # scan_topic = rospy.get_param("~scan_topic")
    
    wall_filter = WallFilter("scan")
    
    rospy.spin()