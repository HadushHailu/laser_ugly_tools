#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>



class ScanFidelityProjection{
    private:        
        ros::NodeHandle node_handle;
        ros::Subscriber scan_sub;
        ros::Publisher pc_pub;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

    public:
        ScanFidelityProjection(ros::NodeHandle nh):node_handle(nh){
                scan_sub = node_handle.subscribe("/scan", 1000, &ScanFidelityProjection::scanCallback,this);
                pc_pub = node_handle.advertise<sensor_msgs::PointCloud>("/scan_pointcloud_map", 1000);
            }

         
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            if(!listener_.waitForTransform(
                    scan_in->header.frame_id,
                    "map",
                    scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                    ros::Duration(1.0))){
                return;
            }

            sensor_msgs::PointCloud cloud;
            projector_.transformLaserScanToPointCloud("map",*scan_in,
            cloud,listener_);

            pc_pub.publish(cloud);

        }

    };


int main(int argc, char **argv)
{
    ros::init(argc,argv,"scan_fidelity_projection");
    ros::NodeHandle n("~");
    ScanFidelityProjection scan_fp(n);
    ros::spin();
    return 0;
}
