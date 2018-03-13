#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <lis_msgs/End_poses.h>
using namespace std;

class MOTION {
  private:
    ros::NodeHandle n;
    ros::Publisher pub_topic;
    ros::Subscriber sub_topic;
    ros::Time current_time;
    lis_msgs::End_poses hand_poses;//publishする手先姿勢データ

  public:
    MOTION() {
        pub_topic = n.advertise<lis_msgs::End_poses>("hand_path_topic", 10);

        hand_poses.right.position.x = 0.70;
        hand_poses.left.position.x  = 0.70;
        hand_poses.right.position.y = -0.38;
        hand_poses.left.position.y  = 0.38;
        hand_poses.right.position.z = 0.35;
        hand_poses.left.position.z  = 0.35;

        /** Hand Orientatin **/
        hand_poses.right.orientation.x = 0.00;
        hand_poses.right.orientation.y = 0.70;
        hand_poses.right.orientation.z = 0.70;
        hand_poses.right.orientation.w = 0.00; 

        hand_poses.left.orientation.x = 0.00;
        hand_poses.left.orientation.y = 0.70;
        hand_poses.left.orientation.z = -0.70;
        hand_poses.left.orientation.w = 0.00;
    }

    bool isReachedGripperRight(tf::StampedTransform tf_gripper, double d) {
        double x = std::abs(tf_gripper.getOrigin().x() - hand_poses.right.position.x) / 0.01;
        double y = std::abs(tf_gripper.getOrigin().y() - hand_poses.right.position.y) / 0.01;
        double z = std::abs(tf_gripper.getOrigin().z() - hand_poses.right.position.z) / 0.01;
        int pos = (int)(sqrt(x + y + z));
        printf("RIGHT: %d \n", pos);
        return (pos < d);
    }

    bool isReachedGripperLeft(tf::StampedTransform tf_gripper, double d) {
        double x = std::abs(tf_gripper.getOrigin().x() - hand_poses.left.position.x) / 0.01;
        double y = std::abs(tf_gripper.getOrigin().y() - hand_poses.left.position.y) / 0.01;
        double z = std::abs(tf_gripper.getOrigin().z() - hand_poses.left.position.z) / 0.01;
        int pos = (int)(sqrt(x + y + z));
        printf("LEFT : %d \n", pos);
        return (pos < d);
    }

    void cloth_handle() {
        tf::TransformListener listener;
        tf::StampedTransform tf_gripper_right, tf_gripper_left;
        ros::Rate loop_rate(10);

        printf("1\n");
        do {
            current_time = ros::Time::now();
            try {
                listener.waitForTransform("/base", ros::Time(0), "right_endgripper2", current_time, "/right_gripper", ros::Duration(1.0));
                listener.lookupTransform("/base",  ros::Time(0), "right_endgripper2", current_time, "/right_gripper", tf_gripper_right);

                listener.waitForTransform("/base", ros::Time(0), "left_endgripper2", current_time, "/left_gripper", ros::Duration(1.0));
                listener.lookupTransform("/base",  ros::Time(0), "left_endgripper2", current_time, "/left_gripper", tf_gripper_left);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            hand_poses.header.frame_id = "/base";
            hand_poses.header.stamp = current_time;
            pub_topic.publish(hand_poses);
            loop_rate.sleep();
        } while (!isReachedGripperLeft(tf_gripper_left, 1) && !isReachedGripperRight(tf_gripper_right, 1));
        // do {
        //     current_time = ros::Time::now();
        //     try {
        //         listener.waitForTransform("/base", ros::Time(0), "right_endgripper2", current_time, "/right_gripper", ros::Duration(1.0));
        //         listener.lookupTransform("/base",  ros::Time(0), "right_endgripper2", current_time, "/right_gripper", tf_gripper_right);
        //         listener.waitForTransform("/base", ros::Time(0), "left_endgripper2", current_time, "/left_gripper", ros::Duration(1.0));
        //         listener.lookupTransform("/base",  ros::Time(0), "left_endgripper2", current_time, "/left_gripper", tf_gripper_left);
        //     } catch (tf::TransformException ex) {
        //         ROS_ERROR("%s",ex.what());
        //     }
        //     hand_poses.header.frame_id = "/base";
        //     hand_poses.header.stamp = current_time;
        //     pub_topic.publish(hand_poses);
        //     loop_rate.sleep();
        // } while (!(isReachedGripperLeft(tf_gripper_left, 0.01) && isReachedGripperRight(tf_gripper_right, 0.01)));
 
        printf("2\n");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_node2");
    cout << "Initializing node... " << endl;

    MOTION motion;
    motion.cloth_handle();
    
    return 0;
}