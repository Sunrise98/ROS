#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <lis_msgs/End_poses.h>
#include <iostream>
#include <string.h>
using namespace std;

class MOTION{
  private:
    ros::NodeHandle n_;
    ros::Publisher pub_topic_;
    ros::Subscriber sub_;
    // lis_msgs::End_poses hand_poses;//publishする手先姿勢データ

  public:
    MOTION() {
        pub_topic_ = n_.advertise<lis_msgs::End_poses>("motion_topic", 10);
        ros::Duration(0.1).sleep(); // sleep for registration of tf

        // hand_poses.right.position.x = 0.70;
        // hand_poses.right.position.y = -0.38;
        // hand_poses.right.position.z = 0.35;
        // hand_poses.right.orientation.x = 0.00;
        // hand_poses.right.orientation.y = 0.70;
        // hand_poses.right.orientation.z = 0.70;
        // hand_poses.right.orientation.w = 0.00; 

        // hand_poses.left.position.x = 0.70;
        // hand_poses.left.position.y = 0.38;
        // hand_poses.left.position.z = 0.35;
        // hand_poses.left.orientation.x = 0.00;
        // hand_poses.left.orientation.y = 0.70;
        // hand_poses.left.orientation.z = -0.70;
        // hand_poses.left.orientation.w = 0.00;
    }

    double get2PointsDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
        return (double) std::sqrt( (double)((x1-x2)*(x1-x2)) + (double)((y1-y2)*(y1-y2)) + (double)((z1-z2)*(z1-z2)) );
    }

    double tfDistance(tf::Transform t1, tf::Transform t2) {
        double x_ = std::pow(t1.getOrigin().x() - t2.getOrigin().x(), 2.0);
        double y_ = std::pow(t1.getOrigin().y() - t2.getOrigin().y(), 2.0);
        double z_ = std::pow(t1.getOrigin().z() - t2.getOrigin().z(), 2.0); 
        return (double) std::sqrt(x_ + y_ + z_);
    }

    void cloth_handle(){
        tf::TransformListener listener;
        tf::StampedTransform tf_hand_right, tf_hand_left, tf_target_right, tf_target_left;

        // ros::Rate loop_rate(0.1); //0.1Hzでfor文を回してデータをpublishする

        // ros::Time current_time;

        // printf("set initial position\n");
        // for(int i = 1; i <= 5; i++) {
        //     printf("%d ", i);
        //     current_time = ros::Time::now();
        //     hand_poses.header.frame_id = "/base";
        //     hand_poses.header.stamp = current_time;
        //     pub_topic_.publish(hand_poses);
        //     loop_rate.sleep();
        // }
        // printf("\n\n");

        while (true) {
            // current_time = ros::Time::now();
            try {
                listener.waitForTransform("now_right", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("now_right", "/base", ros::Time(0), tf_hand_right);

                listener.waitForTransform("now_left", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("now_left", "/base", ros::Time(0), tf_hand_left);

                listener.waitForTransform("target_right", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("target_right", "/base", ros::Time(0), tf_target_right);

                listener.waitForTransform("target_left", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("target_left", "/base", ros::Time(0), tf_target_left);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }

            if (tfDistance(tf_hand_right, tf_target_right) > 0.1) {
                // hand_poses.right.position.y += (double)(tf_target_right.getOrigin().y() - tf_hand_right.getOrigin().y());
                // hand_poses.right.position.z += (double)(tf_target_right.getOrigin().z() - tf_hand_right.getOrigin().z());
                double diff_right_y = (double)(tf_target_right.getOrigin().y() - tf_hand_right.getOrigin().y());
                double diff_right_z = (double)(tf_target_right.getOrigin().z() - tf_hand_right.getOrigin().z());
                printf("RIGHT Y+=%.2lf Z+=%.2lf \n", diff_right_y, diff_right_z);

            }
            if (tfDistance(tf_hand_left, tf_target_left) > 0.1) {
                // hand_poses.left.position.y += (double)(tf_target_left.getOrigin().y() - tf_hand_left.getOrigin().y());
                // hand_poses.left.position.z += (double)(tf_target_left.getOrigin().z() - tf_hand_left.getOrigin().z());
                double diff_left_y = (double)(tf_target_left.getOrigin().y() - tf_hand_left.getOrigin().y());
                double diff_left_z = (double)(tf_target_left.getOrigin().z() - tf_hand_left.getOrigin().z());
                printf("LEFT  Y+=%.2lf Z+=%.2lf \n", diff_left_y, diff_left_z);   
            }
            // if (get2PointsDistance(hand_poses.right.position.x, hand_poses.right.position.y, hand_poses.right.position.z,
            //                        hand_poses.left.position.x, hand_poses.left.position.y, hand_poses.left.position.z) < 0.2) {
            //     break;
            // }  
            // hand_poses.header.frame_id = "/base";
            // hand_poses.header.stamp = current_time;
            // pub_topic_.publish(hand_poses);
            // loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_node_");
    cout << "Initializing node... " << endl;

    MOTION motion;
    motion.cloth_handle();
    
    return 0;
}