#!/usr/bin/env python
import rospy
import math
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from   sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg

class LaserPointCloudEtc():
    def __init__(self):
        # var
        self.lp = lg.LaserProjection()
        self.header = None
        self.fields = None
        self.height = None
        self.width = None
        self.point_step = None
        self.row_step = None
        self.data = None
        self.source_frame = "map"
        self.target_frame = "base_scan"
        self.new_pointcloud = PointCloud2()
        self.pc2_l = None

        # init
        rospy.init_node("laser_pointcloud_node", anonymous=True)

        # func
        self.tf_mat = None
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # sub/pub
        self.laser_sub = rospy.Subscriber("/scan",LaserScan,self.laser_callback)
        self.pointcloud_pub = rospy.Publisher("/pointcloud",PointCloud2,queue_size=10)
        self.tf_pointcloud_pub = rospy.Publisher("/tf_pointcloud",PointCloud2,queue_size=10)


    def laser_callback(self,msg):
        #..
        #rospy.loginfo("Laser info: data_size {0}".format(len(msg.ranges)))
        pc2_msg = self.lp.projectLaser(msg)
        self.pointcloud_pub.publish(pc2_msg)
 
        self.header = pc2_msg.header
        self.fields = pc2_msg.fields
        self.height = pc2_msg.height
        self.width = pc2_msg.width
        self.point_step = pc2_msg.point_step
        self.row_step = pc2_msg.row_step
        self.data = pc2_msg.data

        #rospy.loginfo("PointCloud info height:{0} width:{1} point_step:{2} row_step:{3} binary_data_len:{4}".format(self.height, self.width, self.point_step, self.row_step, len(self.data)))
       
        #pc2_data_all = pc2.read_points(pc2_msg, skip_nans=False)
    
        pc2_data = pc2.read_points(pc2_msg,field_names = ("x", "y", "z"), skip_nans=False)
        self.pc2_l = [x for x in pc2_data]

        #..
        self.new_pointcloud.header = pc2_msg
        rospy.loginfo("PointCloud_data_len: {0}".format(len(self.pc2_l)))

    def transform_pointcloud(self):
        #..
        np_tf_mat = np.array(self.tf_mat)
        np_pcl = np.array(self.pc2_l)
        
        rospy.loginfo("tf_mat shape: {0} pcl shape: {1}".format(np_tf_mat.shape,np_pcl.shape))
        
        #.. add 4 dim
        b = np.ones((np_pcl.shape[0],1))
        np_pcl = np.hstack((np_pcl,b))

        rospy.loginfo("Resized pointcloud: {0}".format(np_pcl.shape))

        np_new_pcl = np.dot(np_pcl,np_tf_mat)
        rospy.loginfo("New TF dot PointCloud_data: {0}".format(np_new_pcl.shape))

        #..
        np_new_pcl = np.delete(np_new_pcl,3,1)
        rospy.loginfo("Deleted PointCloud_data: {0}".format(np_new_pcl.shape))

        new_pcl_header = self.header
        new_pcl_header.frame_id = "map"
        self.new_pointcloud = pc2.create_cloud_xyz32(new_pcl_header,np_new_pcl) 	




    def transformation_matrix(self):
        #..
        try:
            ret_tf = self.tfBuffer.lookup_transform(self.source_frame, self.target_frame,rospy.Time())
            trans = [ret_tf.transform.translation.x,ret_tf.transform.translation.y,ret_tf.transform.translation.z]
            quat   = [ret_tf.transform.rotation.x,ret_tf.transform.rotation.y,ret_tf.transform.rotation.z,ret_tf.transform.rotation.w]
            rot = tf.transformations.euler_from_quaternion(quat)
            self.tf_mat = tf.transformations.compose_matrix(angles=rot,translate=trans)

            #..
            self.transform_pointcloud()
            #rospy.loginfo("tf_mat: {0}".format(self.tf_mat))
        

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)



    def run_it(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.transformation_matrix()
            self.tf_pointcloud_pub.publish(self.new_pointcloud)
            rate.sleep()

if __name__ == "__main__":
    app = LaserPointCloudEtc()
    app.run_it()
