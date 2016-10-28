#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import *
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan

class WallFilter:
    def __init__ (self, scan_topic):

        self.filter_scans = []
        self.filter_median = []
        self.filter_size = 10

        self.scan_topic = scan_topic

        self.filter_scan_pub = rospy.Publisher("filtered_scan", LaserScan, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._rm_walls, self.filter_scan_pub) #needs to be subscribed to wall laser topic

    def _rm_walls(self, data, publisher):
        variance = 0.1

        if not(len(self.filter_scans) < self.filter_size):
            #print("publishing new scan")
            # Update Filter
            self.update_filter(data)

            filtered_ranges = []
            
            for i in xrange(len(self.filter_median)):
                try:
                    if data.ranges[i] < self.filter_median[i]-variance:
                        filtered_ranges.append(data.ranges[i])
                    else:
                        filtered_ranges.append(data.range_max + 1)
                except IndexError:
                    filtered_ranges.append(data.range_max + 1)


            # Publish the new filter
            filtered_scan = data
            h = std_msgs.msg.Header()
            h.stamp = data.header.stamp
            h.frame_id = data.header.frame_id
            filtered_scan.header = h
            filtered_scan.ranges = filtered_ranges
            publisher.publish(filtered_scan)

        else:
            # Add to the filter
            self.filter_scans.append(data.ranges)

    def update_filter(self, data):
            # Remove last thing in filter
            self.filter_scans.pop(0)
            # Add new thing
            self.filter_scans.append(data.ranges)

            re_org = map(list, zip(*self.filter_scans))
            self.filter_median = [numpy.median(z) for z in re_org]
            

if __name__ == '__main__':
    
    rospy.init_node("wall_filter", anonymous=False)

    # scan_topic = rospy.get_param("~scan_topic")
    
    wall_filter = WallFilter("scan")
    
    rospy.spin()