#!/usr/bin/env python
import rospy
import math
import numpy as np
import geometry_msgs.msg
from   sensor_msgs.msg import LaserScan, PointCloud

class PointCloudInfoEtc():
    def __init__(self):
        # var
        self.header = None
        self.points = None
        self.channels = None
        # init
        rospy.init_node("laser_pointcloud_node", anonymous=True)
 
        # sub/pub
        self.pointcloud_sub = rospy.Subscriber("/scan_pointcloud_map",PointCloud, self.pointcloud_callback)

    def pointcloud_callback(self, msg):
        #..
        self.header = msg.header
        self.points = msg.points
        self.channels = msg.channels

        rospy.loginfo("len of pointcloud: {0}".format(len(self.points)))
        #rospy.loginfo("pointcloud: {0}".format(self.points))

        pcl_l = [[x.x, x.y, x.z] for x in self.points]
        pcl_np = np.array(pcl_l)
        rospy.loginfo("pointcloud_to_list :{0}".format(pcl_np.shape))
        #rospy.loginfo("pointcloud: {0}".format(pointcloud_l))

    def run_it(self): 
        rospy.spin()

if __name__ == "__main__":
    app = PointCloudInfoEtc()
    app.run_it()
