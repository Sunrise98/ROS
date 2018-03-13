#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/approximate_voxel_grid.h>//for box filter
#include <pcl/filters/passthrough.h>// for XYZ-range filter
#include <visualization_msgs/Marker.h>
#include <lis_msgs/End_poses.h>
#include <iostream>
#include <string.h>
using namespace std;
typedef pcl::PointXYZ PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLPointCloud;
#define BOX_SIZE 0.01f //[m] 
#define POSE_OFFSET_Z -0.05//[m]

class ROS2PCL{
  private:
    ros::NodeHandle n_;
    ros::Publisher pub_topic_;
    ros::Publisher pub_box_topic_;
    // ros::Publisher pub_hand;
    // ros::Publisher pub_filtered_points_;
    ros::Subscriber sub_;

    // lis_msgs::End_poses hand_poses;//publishする手先姿勢データ

    geometry_msgs::PoseStamped pub_position_;
    // sensor_msgs::PointCloud2Ptr pub_filtered_;
    // pcl::PointCloud<PCLPointType>::Ptr cloud_filtered_;
    // tf::TransformListener tflistener_;
    visualization_msgs::Marker box;
    
  public:
    ROS2PCL(){
        // pub_filtered_points_ = n_.advertise<sensor_msgs::PointCloud2> ("filtered_points", 1);
        // pub_topic_ = n_.advertise<geometry_msgs::PoseStamped> ("position_topic", 1);
        pub_box_topic_ = n_.advertise<visualization_msgs::Marker>("display_object_box", 1);
        sub_ = n_.subscribe("/cloth_cloud/color_filter", 1, &ROS2PCL::callBack, this);
        // pub_filtered_.reset (new sensor_msgs::PointCloud2);
        // cloud_filtered_.reset (new PCLPointCloud);
        ros::Duration(0.5).sleep(); // sleep for registration of tf
        
        box.header.frame_id = pub_position_.header.frame_id = "/kinect2_rgb_optical_frame";

        // pub_hand = n_.advertise<lis_msgs::End_poses>("hand_path_topic", 10);
      
        //決め打ちの右手の姿勢
        // hand_poses.right.position.x = 0.656982770038;
        // hand_poses.right.position.y = -0.852598021641;
        // hand_poses.right.position.z = 0.0388609422173;
        // hand_poses.right.orientation.x = 0.367048116303;
        // hand_poses.right.orientation.y = 0.885911751787;
        // hand_poses.right.orientation.z = -0.108908281936;
        // hand_poses.right.orientation.w = 0.261868353356;


        // tf::Quaternion q;
        // q.setRPY(0.0*M_PI/180.0, 0.0*M_PI/180.0, 90.0*M_PI/180);
        // pub_position_.pose.orientation.x = q.x();
        // pub_position_.pose.orientation.y = q.y();
        // pub_position_.pose.orientation.z = q.z();
        // pub_position_.pose.orientation.w = q.w();
	   
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        box.type = visualization_msgs::Marker::CUBE;
        // Set the marker action.  Options are ADD and DELETE
        box.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        box.pose.orientation.x = 0.0;
        box.pose.orientation.y = 0.0;
        box.pose.orientation.z = 0.0;
        box.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        box.scale.x = 0.05;
        box.scale.z = 0.05;
        box.scale.y = 0.05;
        // Set the color -- be sure to set alpha to something non-zero!
        box.color.r = 0.3f;
        box.color.g = 0.3f;
        box.color.b = 0.0f;
        box.color.a = 0.5f;
        box.lifetime = ros::Duration();         
    }

    //http://wiki.ros.org/ja/pcl/Tutorials
    void callBack(const sensor_msgs::PointCloud2ConstPtr& msg){
        // pcl::fromROSMsg (*msg, *cloud_filtered_);// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud

        // pub_position_.header.stamp = ros::Time::now();
        box.header.stamp = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.points.size () < 5000) {
            //*
            pub_position_.pose.position.x = 0.0;
            pub_position_.pose.position.y = 0.0-0.03;
            pub_position_.pose.position.z = 0.0;
            pub_topic_.publish (pub_position_);
            //*/
            //*
            box.pose.position.x = 0.0;
            box.pose.position.y = 0.0;
            box.pose.position.z = 0.0;
            pub_topic_.publish (pub_position_);
            pub_box_topic_.publish(box);
            //*/
            return;
        }
    
        double x_ave = 0.0, y_ave = 0.0, z_ave = 0.0;

        for (size_t i = 0; i < cloud.points.size (); i++) {
            x_ave += (double) cloud.points[i].x;
            y_ave += (double) cloud.points[i].y;
            z_ave += (double) cloud.points[i].z;
        }
        x_ave /= cloud.points.size ();
        y_ave /= cloud.points.size ();
        z_ave /= cloud.points.size ();

        double w_min, w_max, h_min, h_max, d_min, d_max;
        w_min = w_max = cloud.points[0].x;
        h_min = h_max = cloud.points[0].y;
        d_min = d_max = cloud.points[0].z;

        for (size_t i = 1; i < cloud.points.size (); i++) {
            // if (abs(cloud.points[i].x - x_ave) <= 0.15 /* width */ && abs(cloud.points[i].z - z_ave) <= 0.15 /* depth */ && abs(cloud.points[i].y - y_ave) <= 0.15 /* height */) {
                // width
                if (cloud.points[i].x < w_min)      w_min = cloud.points[i].x;
                else if (cloud.points[i].x > w_max) w_max = cloud.points[i].x;
                // height
                if (cloud.points[i].y < h_min)      h_min = cloud.points[i].y;
                else if(cloud.points[i].y > h_max)  h_max = cloud.points[i].y;
                // depth
                if (cloud.points[i].z < d_min)      d_min = cloud.points[i].z;
                else if(cloud.points[i].z > d_max)  d_max = cloud.points[i].z; 
            // } 
        } 
        double width = (double) abs(w_max - w_min);
        double height= (double) abs(h_max - h_min);
        double depth = (double) abs(d_max - d_min);

        ROS_INFO("cloud size : %d \n", (int)cloud.points.size());

        box.scale.x = width;
        box.scale.y = height;
        box.scale.z = depth;

        box.pose.position.x = x_ave;
        box.pose.position.y = y_ave;
        box.pose.position.z = z_ave;

        pub_box_topic_.publish(box);
        
        // pub_position_.pose.position.x = cloud.points[top_num].x;
        // pub_position_.pose.position.y = cloud.points[top_num].y - 0.03;
        // pub_position_.pose.position.z = cloud.points[top_num].z;

        // double x_top = cloud.points[top_num].x;
        // double y_top = cloud.points[top_num].y;
        // double z_top = cloud.points[top_num].z;

        // pub_topic_.publish (pub_position_);
        // pub_box_topic_.publish(box);
        // pcl::toROSMsg (*cloud_filtered_, *pub_filtered_);//Convert pcl/PointCloud to sensor_msgs/PointCloud2
        // pub_filtered_points_.publish (pub_filtered_);

        // while(n_.ok()){}
    }

    /*
    void filterBOX(void){
        pcl::ApproximateVoxelGrid<PCLPointType> sor;

        sor.setInputCloud (cloud_filtered_);
        sor.setLeafSize (BOX_SIZE, BOX_SIZE, BOX_SIZE);//Down sampling the Pointclouds as 0.01ｍ box
        sor.filter (*cloud_filtered_);
    }

    void filterXYZ(string axis, const double min, const double max){
        pcl::PassThrough<PCLPointType> pass;

        pass.setInputCloud (cloud_filtered_);
        pass.setFilterFieldName (axis);// filtering by z-range
        pass.setFilterLimits (min, max);//[m]
        pass.filter (*cloud_filtered_);
    }

    //Transform coordine of cloud_filtered_ points
    void transformCloud(void){
    	   sensor_msgs::PointCloud2Ptr smpc2_native_cloud (new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Ptr smpc2_transformed_cloud (new sensor_msgs::PointCloud2);

        pcl::toROSMsg (*cloud_filtered_, *smpc2_native_cloud);

        tflistener_.waitForTransform("/base", cloud_filtered_->header.frame_id, cloud_filtered_->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud("/base", *smpc2_native_cloud, *smpc2_transformed_cloud, tflistener_);

        pcl::fromROSMsg (* smpc2_transformed_cloud, *cloud_filtered_);
    }
    */
};

int main(int argc, char **argv){
    ros::init(argc, argv, "detection_node");
    cout << "Initializing node... " << endl;

    ROS2PCL ros2pcl;
    ros::spin();
    
    return 0;
}