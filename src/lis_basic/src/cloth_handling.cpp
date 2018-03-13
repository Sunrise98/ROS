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
        tf::StampedTransform tf_robot_r, tf_robot_l, tf_robot_c;
        tf::StampedTransform tf_human_r, tf_human_l, tf_human_c;
        tf::StampedTransform tf_bound_r, tf_bound_l;
        tf::TransformListener listener;
        ros::Rate loop_rate(20); //0.3Hzでfor文を回してデータをpublishする

        while(true) {
            ros::Time current_time = ros::Time::now();

            try {
                listener.waitForTransform("cloth_point_robot_right", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_robot_right", "/base", ros::Time(0), tf_robot_r);

                listener.waitForTransform("cloth_point_robot_center", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_robot_center", "/base", ros::Time(0), tf_robot_c);

                listener.waitForTransform("cloth_point_robot_left", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_robot_left", "/base", ros::Time(0), tf_robot_l);

                listener.waitForTransform("cloth_point_human_right", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_human_right", "/base", ros::Time(0), tf_human_r);

                listener.waitForTransform("cloth_point_human_center", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_human_center", "/base", ros::Time(0), tf_human_c);

                listener.waitForTransform("cloth_point_human_left", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_human_left", "/base", ros::Time(0), tf_human_l);

                listener.waitForTransform("cloth_point_boundary_right", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_boundary_right", "/base", ros::Time(0), tf_bound_r);

                listener.waitForTransform("cloth_point_boundary_left", "/base", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("cloth_point_boundary_left", "/base", ros::Time(0), tf_bound_l);
                    
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            printf("cloth_point_robot_right  x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_robot_r.getOrigin().x(), tf_robot_r.getOrigin().y(), tf_robot_r.getOrigin().z());
            printf("cloth_point_robot_center x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_robot_c.getOrigin().x(), tf_robot_c.getOrigin().y(), tf_robot_c.getOrigin().z());
            printf("cloth_point_robot_left   x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_robot_l.getOrigin().x(), tf_robot_l.getOrigin().y(), tf_robot_l.getOrigin().z());
            printf("cloth_point_human_right  x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_human_r.getOrigin().x(), tf_human_r.getOrigin().y(), tf_human_r.getOrigin().z());
            printf("cloth_point_human_center x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_human_c.getOrigin().x(), tf_human_c.getOrigin().y(), tf_human_c.getOrigin().z());
            printf("cloth_point_human_left   x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_human_l.getOrigin().x(), tf_human_l.getOrigin().y(), tf_human_l.getOrigin().z());
            printf("cloth_point_bound_right  x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_bound_r.getOrigin().x(), tf_bound_r.getOrigin().y(), tf_bound_r.getOrigin().z());
            printf("cloth_point_bound_left   x:%.2lf\t y:%.2lf\t z%.2lf\n", tf_bound_l.getOrigin().x(), tf_bound_l.getOrigin().y(), tf_bound_l.getOrigin().z());     

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
