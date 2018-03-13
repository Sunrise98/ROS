#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <lis_msgs/End_poses.h>
using namespace std;

class Test{
private:
  ros::NodeHandle n;
  ros::Publisher pub_topic;
  ros::Subscriber sub_topic;
  lis_msgs::End_poses hand_poses;//publishする手先姿勢データ
    
public:
    Test(){
       pub_topic = n.advertise<lis_msgs::End_poses>("hand_path_topic", 10);

       /** kihon **/
       hand_poses.right.position.x = 0.70;
       hand_poses.right.position.y = -0.38;
       hand_poses.right.position.z = 0.35;
       hand_poses.right.orientation.x = 0.00;
       hand_poses.right.orientation.y = 0.70;
       hand_poses.right.orientation.z = 0.70;
       hand_poses.right.orientation.w = 0.00; 

       hand_poses.left.position.x = 0.70;
       hand_poses.left.position.y = 0.38;
       hand_poses.left.position.z = 0.35;
       hand_poses.left.orientation.x = 0.00;
       hand_poses.left.orientation.y = 0.70;
       hand_poses.left.orientation.z = -0.70;
       hand_poses.left.orientation.w = 0.00;

       /** hidari kaiten **/

       // hand_poses.right.position.x = 0.70;
       // hand_poses.right.position.y = 0.0;
       // hand_poses.right.position.z = 0.70;
       // hand_poses.right.orientation.x = 0.00;
       // hand_poses.right.orientation.y = 0.70;
       // hand_poses.right.orientation.z = 0.70;
       // hand_poses.right.orientation.w = 0.00; 

       // hand_poses.left.position.x = 0.70;
       // hand_poses.left.position.y = 0.0;
       // hand_poses.left.position.z = 0.00;
       // hand_poses.left.orientation.x = 0.00;
       // hand_poses.left.orientation.y = 0.70;
       // hand_poses.left.orientation.z = -0.70;
       // hand_poses.left.orientation.w = 0.00;

       /**  migi kaiten **/
       // hand_poses.right.position.x = 0.70;
       // hand_poses.right.position.y = 0.0;
       // hand_poses.right.position.z = 0.00;
       // hand_poses.right.orientation.x = 0.00;
       // hand_poses.right.orientation.y = 0.70;
       // hand_poses.right.orientation.z = 0.70;
       // hand_poses.right.orientation.w = 0.00; 

       // hand_poses.left.position.x = 0.70;
       // hand_poses.left.position.y = 0.0;
       // hand_poses.left.position.z = 0.70;
       // hand_poses.left.orientation.x = 0.00;
       // hand_poses.left.orientation.y = 0.70;
       // hand_poses.left.orientation.z = -0.70;
       // hand_poses.left.orientation.w = 0.00;

    }

    double get2PointsDistance(double x1, double y1, double z1, 
                              double x2, double y2, double z2) {
        return (double) std::sqrt(std::pow(x1-x2, 2.0) + std::pow(y1-y2, 2.0) + std::pow(z1-z2, 2.0));
    }

    bool isInSpace(double x1, double y1, double z1, 
                   double x2, double y2, double z2, double r) {
        double dist = get2PointsDistance(x1, y1, z1, x2, y2, z2);
        printf("%lf \n", dist);
        return (dist < r);
    }

    void gen_path(){
        tf::StampedTransform tf_right_hand, tf_left_hand;
        tf::TransformListener listener;
        ros::Rate loop_rate(0.3); //0.3Hzでfor文を回してデータをpublishする

        int cnt = 0;
        for(int i = 0; i < 5; i++) {
            ros::Time current_time = ros::Time::now();

            try {
                listener.waitForTransform("right_hand", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("right_hand", "/base", ros::Time(0), tf_right_hand);

                listener.waitForTransform("left_hand", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("left_hand", "/base", ros::Time(0), tf_left_hand);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
        
            // bool right_reach = isInSpace(tf_right_hand.getOrigin().x(), tf_right_hand.getOrigin().y(), tf_right_hand.getOrigin().z(), 
            //                              hand_poses.right.position.x, hand_poses.right.position.y, hand_poses.right.position.z, 
            //                              0.15);
            // bool left_reach = isInSpace(tf_left_hand.getOrigin().x(), tf_left_hand.getOrigin().y(), tf_left_hand.getOrigin().z(), 
            //                              hand_poses.left.position.x, hand_poses.left.position.y, hand_poses.left.position.z, 
            //                              0.15);
            // printf("\n");
            // if (right_reach && left_reach) {
            //     if (cnt > 0) break;
            //     cnt ++;  
            // }

            //base座標系でみた目標手先位置の設定
            hand_poses.header.frame_id = "/base";//このデータが、どの座標系から見たものか代入 ここでは/base
            hand_poses.header.stamp = current_time;//このデータがいつの時刻のものか代入
            
            pub_topic.publish(hand_poses);
            loop_rate.sleep();
            
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node_");

    cout << "Initializing node... " << endl;
    Test test;
    test.gen_path();

    return 0;
}
