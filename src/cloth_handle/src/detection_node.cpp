#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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

typedef struct Coord {
    double x, y, z;
} coord;

class DETECTION{
  private:
    ros::NodeHandle n_;
    ros::Publisher pub_topic_;
    ros::Publisher pub_boundary_surface_topic_;
    ros::Publisher pub_boundary_right_topic_;
    ros::Publisher pub_boundary_left_topic_;
    ros::Publisher pub_human_right_topic_;
    ros::Publisher pub_human_center_topic_;
    ros::Publisher pub_human_left_topic_;
    ros::Publisher pub_baxter_right_topic_;
    ros::Publisher pub_baxter_center_topic_;
    ros::Publisher pub_baxter_left_topic_;
    ros::Subscriber sub_;

    /** tf **/
    tf::TransformListener tflistener;
    tf::TransformBroadcaster tfbroadcaster;

    tf::Transform tf_robot_r, tf_robot_l, tf_robot_c;
    tf::Transform tf_human_r, tf_human_l, tf_human_c;
    tf::Transform tf_bound_r, tf_bound_l;
    tf::Quaternion q;

    geometry_msgs::PoseStamped pub_position_;
    visualization_msgs::Marker boundary_surface;
    visualization_msgs::Marker boundary_right;
    visualization_msgs::Marker boundary_left;
    visualization_msgs::Marker baxterSide_right;
    visualization_msgs::Marker baxterSide_center;
    visualization_msgs::Marker baxterSide_left;
    visualization_msgs::Marker humanSide_right;
    visualization_msgs::Marker humanSide_center;
    visualization_msgs::Marker humanSide_left;

  public:
    DETECTION(){
        pub_boundary_surface_topic_ = n_.advertise<visualization_msgs::Marker>("disp_boundary_surface", 1);
        pub_boundary_right_topic_   = n_.advertise<visualization_msgs::Marker>("disp_boundary_right", 1);
        pub_boundary_left_topic_    = n_.advertise<visualization_msgs::Marker>("disp_boundary_left", 1);
        pub_human_right_topic_      = n_.advertise<visualization_msgs::Marker>("disp_human_right", 1);
        pub_human_center_topic_     = n_.advertise<visualization_msgs::Marker>("disp_human_center", 1);
        pub_human_left_topic_       = n_.advertise<visualization_msgs::Marker>("disp_human_left", 1);
        pub_baxter_right_topic_     = n_.advertise<visualization_msgs::Marker>("disp_baxter_right", 1);
        pub_baxter_center_topic_    = n_.advertise<visualization_msgs::Marker>("disp_baxter_center", 1);
        pub_baxter_left_topic_      = n_.advertise<visualization_msgs::Marker>("disp_baxter_left", 1);
        sub_ = n_.subscribe("/cloth_cloud_node/cloth_pointcloud", 1, &DETECTION::callBack, this);
        ros::Duration(0.1).sleep(); // sleep for registration of tf
        
        pub_position_.header.frame_id = "/base";
        boundary_surface.header.frame_id  = pub_position_.header.frame_id;
        boundary_right.header.frame_id    = pub_position_.header.frame_id;
        boundary_left.header.frame_id     = pub_position_.header.frame_id;
        baxterSide_right.header.frame_id  = pub_position_.header.frame_id;
        baxterSide_center.header.frame_id = pub_position_.header.frame_id;
        baxterSide_left.header.frame_id   = pub_position_.header.frame_id;
        humanSide_right.header.frame_id   = pub_position_.header.frame_id;
        humanSide_center.header.frame_id  = pub_position_.header.frame_id;
        humanSide_left.header.frame_id    = pub_position_.header.frame_id;

        boundary_surface.type = visualization_msgs::Marker::CUBE;
        boundary_right.type   = visualization_msgs::Marker::CYLINDER;
        boundary_left.type    = visualization_msgs::Marker::CYLINDER;
        baxterSide_right.type = visualization_msgs::Marker::CUBE;
        baxterSide_center.type= visualization_msgs::Marker::CUBE;
        baxterSide_left.type  = visualization_msgs::Marker::CUBE;
        humanSide_right.type  = visualization_msgs::Marker::SPHERE;
        humanSide_center.type = visualization_msgs::Marker::SPHERE;
        humanSide_left.type   = visualization_msgs::Marker::SPHERE;

        // Set the marker action.  Options are ADD and DELETE
        boundary_surface.action = visualization_msgs::Marker::ADD;
        boundary_right.action   = visualization_msgs::Marker::ADD;
        boundary_left.action    = visualization_msgs::Marker::ADD;
        baxterSide_right.action = visualization_msgs::Marker::ADD;
        baxterSide_center.action= visualization_msgs::Marker::ADD;
        baxterSide_left.action  = visualization_msgs::Marker::ADD;
        humanSide_right.action  = visualization_msgs::Marker::ADD;
        humanSide_center.action = visualization_msgs::Marker::ADD;
        humanSide_left.action   = visualization_msgs::Marker::ADD;
        
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        boundary_surface.pose.orientation.x = 0.0;
        boundary_surface.pose.orientation.y = 0.0;
        boundary_surface.pose.orientation.z = 0.0;
        boundary_surface.pose.orientation.w = 1.0;
        boundary_right.pose.orientation.x = 0.0;
        boundary_right.pose.orientation.y = 0.0;
        boundary_right.pose.orientation.z = 0.0;
        boundary_right.pose.orientation.w = 1.0;
        boundary_left.pose.orientation.x = 0.0;
        boundary_left.pose.orientation.y = 0.0;
        boundary_left.pose.orientation.z = 0.0;
        boundary_left.pose.orientation.w = 1.0;
        baxterSide_right.pose.orientation.x = 0.0;
        baxterSide_right.pose.orientation.y = 0.0;
        baxterSide_right.pose.orientation.z = 0.0;
        baxterSide_right.pose.orientation.w = 1.0;
        baxterSide_center.pose.orientation.x = 0.0;
        baxterSide_center.pose.orientation.y = 0.0;
        baxterSide_center.pose.orientation.z = 0.0;
        baxterSide_center.pose.orientation.w = 1.0;
        baxterSide_left.pose.orientation.x  = 0.0;
        baxterSide_left.pose.orientation.y  = 0.0;
        baxterSide_left.pose.orientation.z  = 0.0;
        baxterSide_left.pose.orientation.w  = 1.0;
        humanSide_right.pose.orientation.x  = 0.0;
        humanSide_right.pose.orientation.y  = 0.0;
        humanSide_right.pose.orientation.z  = 0.0;
        humanSide_right.pose.orientation.w  = 1.0;
        humanSide_center.pose.orientation.x  = 0.0;
        humanSide_center.pose.orientation.y  = 0.0;
        humanSide_center.pose.orientation.z  = 0.0;
        humanSide_center.pose.orientation.w  = 1.0;
        humanSide_left.pose.orientation.x   = 0.0;
        humanSide_left.pose.orientation.y   = 0.0;
        humanSide_left.pose.orientation.z   = 0.0;
        humanSide_left.pose.orientation.w   = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        boundary_surface.scale.x    = 0.05;
        boundary_surface.scale.z    = 0.05;
        boundary_surface.scale.y    = 0.05;
        boundary_right.scale.x      = 0.05;
        boundary_right.scale.z      = 0.05;
        boundary_right.scale.y      = 0.05;
        boundary_left.scale.x       = 0.05;
        boundary_left.scale.z       = 0.05;
        boundary_left.scale.y       = 0.05;
        baxterSide_right.scale.x    = 0.05;
        baxterSide_right.scale.z    = 0.05;
        baxterSide_right.scale.y    = 0.05;
        baxterSide_center.scale.x   = 0.05;
        baxterSide_center.scale.z   = 0.05;
        baxterSide_center.scale.y   = 0.05;
        baxterSide_left.scale.x     = 0.05;
        baxterSide_left.scale.z     = 0.05;
        baxterSide_left.scale.y     = 0.05;
        humanSide_right.scale.x     = 0.05;
        humanSide_right.scale.z     = 0.05;
        humanSide_right.scale.y     = 0.05;
        humanSide_center.scale.x    = 0.05;
        humanSide_center.scale.z    = 0.05;
        humanSide_center.scale.y    = 0.05;
        humanSide_left.scale.x      = 0.05;
        humanSide_left.scale.z      = 0.05;
        humanSide_left.scale.y      = 0.05;
        // Set the color -- be sure to set alpha to something non-zero!
        boundary_surface.color.r    = 1.0f;
        boundary_surface.color.g    = 0.0f;
        boundary_surface.color.b    = 0.0f;
        boundary_surface.color.a    = 0.8f;
        humanSide_center.color.r    = 0.0f;
        humanSide_center.color.g    = 0.0f;
        humanSide_center.color.b    = 0.0f;
        humanSide_center.color.a    = 1.0f;
        baxterSide_center.color.r   = 0.0f;
        baxterSide_center.color.g   = 0.0f;
        baxterSide_center.color.b   = 0.0f;
        baxterSide_center.color.a   = 1.0f;
        
        boundary_right.color.r      = 0.0f;
        boundary_right.color.g      = 0.0f;
        boundary_right.color.b      = 0.0f;
        boundary_right.color.a      = 1.0f;
        boundary_left.color.r       = 0.0f;
        boundary_left.color.g       = 0.0f;
        boundary_left.color.b       = 0.0f;
        boundary_left.color.a       = 1.0f;

        baxterSide_right.color.r    = 0.0f;
        baxterSide_right.color.g    = 0.0f;
        baxterSide_right.color.b    = 1.0f;
        baxterSide_right.color.a    = 1.0f;
        baxterSide_left.color.r     = 0.5f;
        baxterSide_left.color.g     = 0.5f;
        baxterSide_left.color.b     = 0.0f;
        baxterSide_left.color.a     = 1.0f;
        humanSide_right.color.r     = 0.5f;
        humanSide_right.color.g     = 0.5f;
        humanSide_right.color.b     = 0.0f;
        humanSide_right.color.a     = 1.0f;
        humanSide_left.color.r      = 0.0f;
        humanSide_left.color.g      = 0.0f;
        humanSide_left.color.b      = 1.0f;
        humanSide_left.color.a      = 1.0f;
        
        // lifetime 
        boundary_surface.lifetime   = ros::Duration();
        boundary_right.lifetime     = ros::Duration();
        boundary_left.lifetime      = ros::Duration();
        baxterSide_right.lifetime   = ros::Duration();
        baxterSide_center.lifetime  = ros::Duration();
        baxterSide_left.lifetime    = ros::Duration();
        humanSide_right.lifetime    = ros::Duration();
        humanSide_center.lifetime   = ros::Duration();
        humanSide_left.lifetime     = ros::Duration();    
    }

    //http://wiki.ros.org/ja/pcl/Tutorials
    void callBack(const sensor_msgs::PointCloud2ConstPtr& msg){
        pub_position_.header.stamp      = ros::Time::now();
        boundary_surface.header.stamp   = ros::Time::now();
        boundary_right.header.stamp     = ros::Time::now();
        boundary_left.header.stamp      = ros::Time::now();
        baxterSide_right.header.stamp   = ros::Time::now();
        baxterSide_center.header.stamp  = ros::Time::now();
        baxterSide_left.header.stamp    = ros::Time::now();
        humanSide_right.header.stamp    = ros::Time::now();
        humanSide_center.header.stamp   = ros::Time::now();
        humanSide_left.header.stamp     = ros::Time::now();
        
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.points.size () < 100) {
            boundary_surface.pose.position.x = 0.0;
            boundary_surface.pose.position.y = 0.0;
            boundary_surface.pose.position.z = 0.0;
            pub_topic_.publish (pub_position_);
            pub_boundary_surface_topic_.publish(boundary_surface);
            return;
        }
        
        double w_min, w_max, h_min, h_max, d_min, d_max;
        h_min = h_max = cloud.points[0].x;
        w_min = w_max = cloud.points[0].y;
        d_min = d_max = cloud.points[0].z;
        for (size_t i = 0; i < cloud.points.size(); i++) {
            // width
            if (cloud.points[i].y < w_min)      w_min = cloud.points[i].y;
            else if (cloud.points[i].y > w_max) w_max = cloud.points[i].y;
            // height
            if (cloud.points[i].x < h_min)      h_min = cloud.points[i].x;
            else if(cloud.points[i].x > h_max)  h_max = cloud.points[i].x;
            // depth
            if (cloud.points[i].z < d_min)      d_min = cloud.points[i].z;
            else if(cloud.points[i].z > d_max)  d_max = cloud.points[i].z; 
        }
        double x_ave = (double) ((h_min + h_max) / 2);
        double y_ave = (double) ((w_min + w_max) / 2);
        double z_ave = (double) ((d_min + d_max) / 2);

        double width = (double) abs(w_max - w_min);
        double height= (double) abs(h_max - h_min);
        double depth = (double) abs(d_max - d_min);

        ROS_INFO("cloud size : %d", (int)cloud.points.size());
        ROS_INFO("w_min: %.2lf \tw_max: %.2lf \twidth : %.2lf", w_min, w_max, width);
        ROS_INFO("h_min: %.2lf \th_max: %.2lf \theight: %.2lf", h_min, h_max, height);
        ROS_INFO("d_min: %.2lf \td_max: %.2lf \tdepth : %.2lf", d_min, d_max, depth);
        printf("\n");
        ROS_INFO("x_ave: %.2lf \ty_ave: %.2lf \tz_ave: %.2lf", x_ave, y_ave, z_ave);
        printf("\n");

        boundary_surface.scale.x = 0.001;
        boundary_surface.scale.y = width + 0.05;
        boundary_surface.scale.z = depth + 0.05;
        boundary_surface.pose.position.x = x_ave;
        boundary_surface.pose.position.y = y_ave;
        boundary_surface.pose.position.z = z_ave;

        baxterSide_right.pose.position.x = baxterSide_left.pose.position.x = x_ave;
        humanSide_right.pose.position.x  = humanSide_left.pose.position.x  = x_ave;
        double human_right, human_left, baxter_right, baxter_left, baund_left, baund_right;
        human_right = baxter_left = baund_left = w_min;
        human_left  = baxter_right = baund_right = w_max;
        for (size_t i = 0; i < cloud.points.size (); i++) {
            int r = cloud.points[i].r;
            int g = cloud.points[i].g;
            int b = cloud.points[i].b;
            if(g > 127 && g > (r+50) && g > (b+50)) {
                if (cloud.points[i].x > (x_ave + height/4 - 0.01) && cloud.points[i].x < (x_ave + height/4 + 0.01)) {
                    if (cloud.points[i].y >= human_right) 
                        human_right = cloud.points[i].y;
                    else if (cloud.points[i].y <= human_left)
                        human_left = cloud.points[i].y; 
                } else if (cloud.points[i].x > (x_ave - height/4 - 0.01) && cloud.points[i].x < (x_ave - height/4 + 0.01)) {
                    if (cloud.points[i].y >= baxter_left) 
                        baxter_left = cloud.points[i].y;
                    else if (cloud.points[i].y <= baxter_right) 
                        baxter_right = cloud.points[i].y;
                } else if (cloud.points[i].x > (x_ave - 0.01) && cloud.points[i].x < (x_ave + 0.01)) {
                    if (cloud.points[i].y >= baund_left) 
                        baund_left = cloud.points[i].y;
                    else if (cloud.points[i].y <= baund_right) 
                        baund_right = cloud.points[i].y;
                }
            }  
        }
        const double baxter_side_x = x_ave - height/4;
        const double human_side_x = x_ave + height/4;
        for (size_t i = 0; i < cloud.points.size (); i++) {
            int r = cloud.points[i].r;
            int g = cloud.points[i].g;
            int b = cloud.points[i].b;
            if(g > 127 && g > (r+50) && g > (b+50)) { // CLOTH CONDITION
                /* human side */
                if (cloud.points[i].x > (human_side_x - 0.01) && cloud.points[i].x < (human_side_x + 0.01) &&
                    cloud.points[i].y > ((human_right + human_left)/2-0.01) && cloud.points[i].y < ((human_right + human_left)/2+0.01)) {
                    humanSide_center.pose.position.x = cloud.points[i].x;
                    humanSide_center.pose.position.y = cloud.points[i].y;
                    humanSide_center.pose.position.z = cloud.points[i].z; 
                } else if (cloud.points[i].x > (human_side_x - 0.01) && cloud.points[i].x < (human_side_x + 0.01) &&
                    cloud.points[i].y > ((human_right + y_ave)/2-0.01) && cloud.points[i].y < ((human_right + y_ave)/2+0.01)) {
                    humanSide_right.pose.position.x = cloud.points[i].x;
                    humanSide_right.pose.position.y = cloud.points[i].y;
                    humanSide_right.pose.position.z = cloud.points[i].z;
                } else if (cloud.points[i].x > (human_side_x - 0.01) && cloud.points[i].x < (human_side_x + 0.01) &&
                    cloud.points[i].y > ((human_left + y_ave)/2-0.01) && cloud.points[i].y < ((human_left + y_ave)/2+0.01)) {
                    humanSide_left.pose.position.x = cloud.points[i].x;
                    humanSide_left.pose.position.y = cloud.points[i].y;
                    humanSide_left.pose.position.z = cloud.points[i].z;
                }
                /* baxter side */
                else if (cloud.points[i].x > (baxter_side_x - 0.01) && cloud.points[i].x < (baxter_side_x + 0.01) &&
                    cloud.points[i].y > ((baxter_right + baxter_left)/2-0.01) && cloud.points[i].y < ((baxter_right + baxter_left)/2+0.01)) {
                    baxterSide_center.pose.position.x = cloud.points[i].x;
                    baxterSide_center.pose.position.y = cloud.points[i].y;
                    baxterSide_center.pose.position.z = cloud.points[i].z;
                } else if (cloud.points[i].x > (baxter_side_x - 0.01) && cloud.points[i].x < (baxter_side_x + 0.01) &&
                    cloud.points[i].y > ((baxter_right + y_ave)/2-0.01) && cloud.points[i].y < ((baxter_right + y_ave)/2+0.01)) {
                    baxterSide_right.pose.position.x = cloud.points[i].x;
                    baxterSide_right.pose.position.y = cloud.points[i].y;
                    baxterSide_right.pose.position.z = cloud.points[i].z;
                } else if (cloud.points[i].x > (baxter_side_x - 0.01) && cloud.points[i].x < (baxter_side_x + 0.01) &&
                    cloud.points[i].y > ((baxter_left + y_ave)/2-0.01) && cloud.points[i].y < ((baxter_left + y_ave)/2+0.01)) {
                    baxterSide_left.pose.position.x = cloud.points[i].x;
                    baxterSide_left.pose.position.y = cloud.points[i].y;
                    baxterSide_left.pose.position.z = cloud.points[i].z;
                }
                /* boundary */
                else if (cloud.points[i].x > (x_ave - 0.01) && cloud.points[i].x < (x_ave + 0.01) &&
                    cloud.points[i].y > ((baund_right + y_ave)/2 - 0.01) && cloud.points[i].y < ((baund_right + y_ave)/2 + 0.01)) {
                    boundary_right.pose.position.x = cloud.points[i].x;
                    boundary_right.pose.position.y = cloud.points[i].y;
                    boundary_right.pose.position.z = cloud.points[i].z;
                } else if (cloud.points[i].x > (x_ave - 0.01) && cloud.points[i].x < (x_ave + 0.01) &&
                    cloud.points[i].y > ((baund_left + y_ave)/2 - 0.01) && cloud.points[i].y < ((baund_left + y_ave)/2 + 0.01)) {
                    boundary_left.pose.position.x = cloud.points[i].x;
                    boundary_left.pose.position.y = cloud.points[i].y;
                    boundary_left.pose.position.z = cloud.points[i].z;
                }
            }  
        }

        ROS_INFO("baxterSide_right  x: %.2lf \ty: %.2lf \tz: %.2lf", baxterSide_right.pose.position.x, baxterSide_right.pose.position.y, baxterSide_right.pose.position.z);
        ROS_INFO("baxterSide_center x: %.2lf \ty: %.2lf \tz: %.2lf", baxterSide_center.pose.position.x, baxterSide_center.pose.position.y, baxterSide_center.pose.position.z);
        ROS_INFO("baxterSide_left   x: %.2lf \ty: %.2lf \tz: %.2lf", baxterSide_left.pose.position.x, baxterSide_left.pose.position.y, baxterSide_left.pose.position.z);
        ROS_INFO("humanSide_right   x: %.2lf \ty: %.2lf \tz: %.2lf", humanSide_right.pose.position.x, humanSide_right.pose.position.y, humanSide_right.pose.position.z);
        ROS_INFO("humanSide_center  x: %.2lf \ty: %.2lf \tz: %.2lf", humanSide_center.pose.position.x, humanSide_center.pose.position.y, humanSide_center.pose.position.z);
        ROS_INFO("humanSide_left    x: %.2lf \ty: %.2lf \tz: %.2lf", humanSide_left.pose.position.x, humanSide_left.pose.position.y, humanSide_left.pose.position.z);
        printf("\n");

        pub_boundary_surface_topic_.publish(boundary_surface);
        pub_boundary_right_topic_.publish(boundary_right);
        pub_boundary_left_topic_.publish(boundary_left);
        pub_baxter_right_topic_.publish(baxterSide_right);
        pub_baxter_center_topic_.publish(baxterSide_center);
        pub_baxter_left_topic_.publish(baxterSide_left);
        pub_human_right_topic_.publish(humanSide_right);
        pub_human_center_topic_.publish(humanSide_center);
        pub_human_left_topic_.publish(humanSide_left);

        tf_robot_r.setOrigin(tf::Vector3(baxterSide_right.pose.position.x, baxterSide_right.pose.position.y, baxterSide_right.pose.position.z));
        q.setRPY(0,0,0);
        tf_robot_r.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_robot_r, ros::Time::now(), "/base", "cloth_point_robot_right"));

        tf_robot_c.setOrigin(tf::Vector3(baxterSide_center.pose.position.x, baxterSide_center.pose.position.y, baxterSide_center.pose.position.z));
        q.setRPY(0,0,0);
        tf_robot_c.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_robot_c, ros::Time::now(), "/base", "cloth_point_robot_center"));

        tf_robot_l.setOrigin(tf::Vector3(baxterSide_left.pose.position.x, baxterSide_left.pose.position.y, baxterSide_left.pose.position.z));
        q.setRPY(0,0,0);
        tf_robot_l.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_robot_l, ros::Time::now(), "/base", "cloth_point_robot_left"));

        tf_human_r.setOrigin(tf::Vector3(humanSide_right.pose.position.x, humanSide_right.pose.position.y, humanSide_right.pose.position.z));
        q.setRPY(0,0,0);
        tf_human_r.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_human_r, ros::Time::now(), "/base", "cloth_point_human_right"));

        tf_human_c.setOrigin(tf::Vector3(humanSide_center.pose.position.x, humanSide_center.pose.position.y, humanSide_center.pose.position.z));
        q.setRPY(0,0,0);
        tf_human_c.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_human_c, ros::Time::now(), "/base", "cloth_point_human_center"));

        tf_human_l.setOrigin(tf::Vector3(humanSide_left.pose.position.x, humanSide_left.pose.position.y, humanSide_left.pose.position.z));
        q.setRPY(0,0,0);
        tf_human_l.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_human_l, ros::Time::now(), "/base", "cloth_point_human_left"));

        tf_bound_r.setOrigin(tf::Vector3(boundary_right.pose.position.x, boundary_right.pose.position.y, boundary_right.pose.position.z));
        q.setRPY(0,0,0);
        tf_bound_r.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_bound_r, ros::Time::now(), "/base", "cloth_point_boundary_right"));

        tf_bound_l.setOrigin(tf::Vector3(boundary_left.pose.position.x, boundary_left.pose.position.y, boundary_left.pose.position.z));
        q.setRPY(0,0,0);
        tf_bound_l.setRotation(q);
        tfbroadcaster.sendTransform(tf::StampedTransform(tf_bound_l, ros::Time::now(), "/base", "cloth_point_boundary_left"));

        printf("\n\n");  
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "detection_node");
    cout << "Initializing node... " << endl;

    DETECTION DETECTION;
    ros::spin();
    
    return 0;
}