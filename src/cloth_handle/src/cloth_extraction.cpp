#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
typedef sensor_msgs::PointCloud2 SMPC2;

class ROS2PCL{
  private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    sensor_msgs::PointCloud2 output;
    tf::TransformListener tflistener;
    
  public:
    ROS2PCL(){
		pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGBA>> ("cloth_pointcloud", 1);
		sub = n.subscribe("kinect2_pointcloud", 5, &ROS2PCL::callBack, this);
    }

    void callBack(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg(*msg, *cloud);
        
        double x1 = 0.5, y1 = 1.0, x2 = -0.5, y2 = -1.0, z1 = 2.0 , z2 = 0.0;
        // filtered (default size)
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits(x2, x1);
        pass.filter (*cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits(y2, y1);
        pass.filter(*cloud);
        pass.setFilterFieldName ("z"); 
        pass.setFilterLimits(z2, z1);
        pass.filter(*cloud);
        printf("1 ");
        // get space of cloth 
        for (size_t i = 0; i < cloud->points.size(); i++) {
            int r = (int) cloud->points[i].r;
            int g = (int) cloud->points[i].g;
            int b = (int) cloud->points[i].b;
            if (g > 127 && g > (r+50) && g > (b+50)) {
                if (x1 > cloud->points[i].x) x1 = cloud->points[i].x;
                if (y1 > cloud->points[i].y) y1 = cloud->points[i].y;
                if (z1 > cloud->points[i].z) z1 = cloud->points[i].z;
                if (x2 < cloud->points[i].x) x2 = cloud->points[i].x;
                if (y2 < cloud->points[i].y) y2 = cloud->points[i].y;
                if (z2 < cloud->points[i].z) z2 = cloud->points[i].z;
            }
        }
        printf("2 ");
        // filtered (space of cloth)
        pass.setInputCloud (cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x1, x2);
        pass.filter(*cloud);
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y1, y2);
        pass.filter(*cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z1, z2);
        pass.filter(*cloud);
        printf("3 ");
        int r_min = 255, r_max = 0;
        int g_min = 255, g_max = 0;
        int b_min = 255, b_max = 0; 
        for (size_t i = 0; i < cloud->points.size(); i++) {
            int r = (int) cloud->points[i].r;
            int g = (int) cloud->points[i].g;
            int b = (int) cloud->points[i].b;
            if (g > 127 && g > (r+50) && g > (b+50)) {
                if (r_min > g-r) r_min = g-r;
                else if (r_max < g-r) r_max = g-r;
                if (g_min > g) g_min = g;
                else if (g_max < g) g_max = g;
                if (b_min > g-b) b_min = g-b;
                else if (b_max < g-b) b_max = g-b; 

                if (x1 > cloud->points[i].x) x1 = cloud->points[i].x;
                if (y1 > cloud->points[i].y) y1 = cloud->points[i].y;
                if (z1 > cloud->points[i].z) z1 = cloud->points[i].z;
                if (x2 < cloud->points[i].x) x2 = cloud->points[i].x;
                if (y2 < cloud->points[i].y) y2 = cloud->points[i].y;
                if (z2 < cloud->points[i].z) z2 = cloud->points[i].z;
            }
        }
        printf("4 ");
        pcl::PointCloud<pcl::PointXYZRGBA> out_cloud;
        out_cloud.height = cloud->height;
        out_cloud.width  = cloud->width;
        out_cloud.is_dense = false;
        out_cloud.points.resize(cloud->height * cloud->width);
        for (size_t i = 0; i < cloud->points.size(); ++i){
            out_cloud.points[i] = cloud->points[i];
        }
        ROS_INFO("cloud size :%d", (int)out_cloud.points.size());
        ROS_INFO("x1: %.2lf \tx2: %.2lf \t size: %.2lf", x1, x2, fabs(x1-x2));
        ROS_INFO("y1: %.2lf \ty2: %.2lf \t size: %.2lf", y1, y2, fabs(y1-y2));
        ROS_INFO("z1: %.2lf \tz2: %.2lf \t size: %.2lf", z1, z2, fabs(z1-z2));
        printf("\n");
        ROS_INFO("r_min: %3d \tr_max: %3d", r_min, r_max);
        ROS_INFO("g_min: %3d \tg_max: %3d", g_min, g_max);
        ROS_INFO("b_min: %3d \tb_max: %3d", b_min, b_max);
        printf("\n\n");
        // publish 
        if (out_cloud.points.size() > 0) {
            pcl::toROSMsg(out_cloud, output);
            output.header.frame_id = "kinect2_rgb_optical_frame";
            output.header.stamp = ros::Time::now();
            // tflistener.waitForTransform("/base", output.header.frame_id, output.header.stamp, ros::Duration(1.0));
            // pcl_ros::transformPointCloud("/base", output, output, tflistener);
            pub.publish(output);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "cloth_extraction_node");
    cout << "Initializing node... " << endl;
    ROS2PCL ros2pcl;
    ros::spin();
    
    return 0;
}
