#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan 

class LaserInfo():
    def __init__(self):
        # var
        self.angle_min = None
        self.angle_max = None
        self.angle_inc = None
        self.time_inc = None
        self.scan_time = None
        self.range_min = None
        self.range_max = None
        self.range = None
        self.inten = None

        # init
        rospy.init_node("laser_info_node", anonymous=True)

        # sub/pub
        self.laser_sub = rospy.Subscriber("/scan",LaserScan,self.laser_callback)

    def laser_callback(self,msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_inc = msg.angle_increment
        self.time_inc = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.range = msg.ranges
        self.inten = msg.intensities

        #rospy.loginfo("Laser info: angle_min: {0} angle_max :{1} angle_inc: {2}".format(self.angle_min, self.angle_max, self.angle_inc,len))
        #rospy.loginfo("Laser info: data size: {0} intensities size:{1} ".format(len(self.range),len(self.inten)))

        angle_min_rad = self.angle_min * (180 / math.pi)
        angle_max_rad = self.angle_max * (180 / math.pi)

        covered_angle = (len(self.range) * self.angle_inc ) * (180 / math.pi)
        #rospy.loginfo("var: {0} angle_inc: {1}".format(covered_angle, self.angle_inc * (180 / math.pi)))

        laser_np = np.array(self.range)
        filter_inf = np.where(np.isinf(laser_np))
        rospy.loginfo("len of inf-ranges: {0}".format(len(laser_np[filter_inf])))

        filter_scan = np.delete(laser_np,filter_inf,None)
        rospy.loginfo("len of float-ranges: {0}".format(len(filter_scan)))
        


    def run_it(self):
        rospy.spin()

if __name__ == "__main__":
    app = LaserInfo()
    app.run_it()
