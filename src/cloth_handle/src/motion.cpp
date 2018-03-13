#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <lis_msgs/End_poses.h>
using namespace std;

#define CLOTH_WIDTH     0.7
#define CLOTH_HEIGHT    1.0

#define FOLD_GAIN       1.5
#define ROTATE_GAIN     1.5

#define P_Hr_x          tf_human_right.getOrigin().x()
#define P_Hr_y          tf_human_right.getOrigin().y()
#define P_Hr_z          tf_human_right.getOrigin().z()
#define P_Hc_x          tf_human_center.getOrigin().x()
#define P_Hc_y          tf_human_center.getOrigin().y()
#define P_Hc_z          tf_human_center.getOrigin().z() 
#define P_Hl_x          tf_human_left.getOrigin().x()
#define P_Hl_y          tf_human_left.getOrigin().y()
#define P_Hl_z          tf_human_left.getOrigin().z()  
#define P_Br_x          tf_bound_right.getOrigin().x()
#define P_Br_y          tf_bound_right.getOrigin().y()
#define P_Br_z          tf_bound_right.getOrigin().z() 
#define P_Bl_x          tf_bound_left.getOrigin().x()
#define P_Bl_y          tf_bound_left.getOrigin().y()
#define P_Bl_z          tf_bound_left.getOrigin().z()
#define P_Rr_x          tf_robot_right.getOrigin().x()
#define P_Rr_y          tf_robot_right.getOrigin().y()
#define P_Rr_z          tf_robot_right.getOrigin().z()
#define P_Rc_x          tf_robot_center.getOrigin().x()
#define P_Rc_y          tf_robot_center.getOrigin().y()
#define P_Rc_z          tf_robot_center.getOrigin().z() 
#define P_Rl_x          tf_robot_left.getOrigin().x()
#define P_Rl_y          tf_robot_left.getOrigin().y()
#define P_Rl_z          tf_robot_left.getOrigin().z()  

class Test{
private:
    ros::NodeHandle n;
    ros::Publisher pub_topic;
    ros::Subscriber sub_topic;
    lis_msgs::End_poses hand_poses;//publishする手先姿勢データ
    lis_msgs::End_poses init_hand_poses[3];//手先姿勢データ(初期状態)
    ros::Time current_time;
    int init_num;

public:
    Test(){
        pub_topic = n.advertise<lis_msgs::End_poses>("hand_path_topic", 10);

        std::cout << "Pease enter the init_num of cloth." << std::endl;
        std::cout << " 0: Transverse plane " << std::endl;
        std::cout << " 1: Sagittal plane (Right is up)" << std::endl;
        std::cout << " 2: Sagittal plane (Left is up)" << std::endl;
        // ハンド姿勢の初期状態を設定       

        // 初期ハンド位置の設定(キーボード入力)
        std::cin >> init_num;

        hand_poses.right.position.x = 0.70;
        hand_poses.left.position.x  = 0.70;
        hand_poses.right.position.y = (init_num > 0) ? 0.00 : - (double)(CLOTH_WIDTH / 2.0);
        hand_poses.left.position.y  = (init_num > 0) ? 0.00 : (double)(CLOTH_WIDTH / 2.0);

        hand_poses.right.position.z = (init_num == 0) ? (double)(CLOTH_WIDTH / 2.0) : (init_num == 1) ? (double)CLOTH_WIDTH : 0.00;
        hand_poses.left.position.z  = (init_num == 0) ? (double)(CLOTH_WIDTH / 2.0) : (init_num == 1) ? 0.00 : (double)CLOTH_WIDTH;

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

    /**
     * ハンドの手先位置が目標座標に達しているかどうか？
     */
    bool isGripperReached (const double d, tf::StampedTransform tf_gripper_left, tf::StampedTransform tf_gripper_right) {
        double dist_right = std::pow(tf_gripper_right.getOrigin().x() - hand_poses.right.position.x, 2.0) +
                            std::pow(tf_gripper_right.getOrigin().y() - hand_poses.right.position.y, 2.0) +
                            std::pow(tf_gripper_right.getOrigin().z() - hand_poses.right.position.z, 2.0);
        dist_right = std::sqrt(dist_right);
        double dist_left  = std::pow(tf_gripper_left.getOrigin().x() - hand_poses.left.position.x, 2.0) +
                            std::pow(tf_gripper_left.getOrigin().y() - hand_poses.left.position.y, 2.0) +
                            std::pow(tf_gripper_left.getOrigin().z() - hand_poses.left.position.z, 2.0);
        dist_left  = std::sqrt(dist_left);
        // std::cout << dist_right <<std::endl;
        // std::cout << dist_left <<std::endl; 
        if (dist_right <= d && dist_left <= d) return true;
        return false;
    }

    /**
     * ハンドの手先姿勢が目標座標に到達するまで"hand_poses"をpublishし続ける
     */
    void moveHandPosition () {
        std::cout << "start" << endl;
        tf::TransformListener listener;
        tf::StampedTransform tf_gripper_left, tf_gripper_right;
        ros::Rate loop_rate(1);
        // ハンド位置の制限
        if (hand_poses.right.position.y < -(CLOTH_WIDTH / 2.0)) {
            hand_poses.right.position.y = -(CLOTH_WIDTH / 2.0);
        } else if (hand_poses.right.position.y > -0.05) {
            hand_poses.right.position.y = -0.05;
        }
        if (hand_poses.left.position.y > (CLOTH_WIDTH / 2.0)) {
            hand_poses.left.position.y = (CLOTH_WIDTH / 2.0);
        } else if (hand_poses.left.position.y < 0.05) {
            hand_poses.left.position.y = 0.05;
        }
        if (hand_poses.right.position.z > CLOTH_WIDTH) {
            hand_poses.right.position.z = CLOTH_WIDTH;
        } else if (hand_poses.right.position.z < -0.00) {
            hand_poses.right.position.z = 0.00;
        }
        if (hand_poses.left.position.z > CLOTH_WIDTH) {
            hand_poses.left.position.z = CLOTH_WIDTH;
        } else if (hand_poses.left.position.z < 0.00) {
            hand_poses.left.position.z = 0.00;
        }
        // 目標座標に到達するまで手先姿勢データをpublishし続ける．
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
        } while (!isGripperReached(0.05, tf_gripper_left, tf_gripper_right));
        std::cout << "end" << endl;
    }

    void gen_path() {
        tf::StampedTransform tf_human_right, tf_human_left, tf_robot_left, tf_robot_right;
        tf::StampedTransform tf_human_center, tf_bound_right, tf_bound_left, tf_robot_center;
        tf::StampedTransform tf_gripper_right, tf_gripper_left;

        tf::TransformListener listener;
        ros::Rate loop_rate(5); //5Hzでfor文を回してデータをpublishする

        std::cout << "1: SET INITIAL HAND POSITION" << std::endl;
        moveHandPosition();
        std::cout << "END" << std::endl;

        ros::Duration(2.5).sleep();
        std::cout << "2: SYNMMETRIC MOTION" << std::endl;
        ros::Duration(2.5).sleep();

        while(ros::ok()) {
            current_time = ros::Time::now();
            try {
                listener.waitForTransform("/base", ros::Time(0), "cloth_point_robot_right", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_robot_right",     "/base", ros::Time(0), tf_robot_right);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_robot_center", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_robot_center",    "/base", ros::Time(0), tf_robot_center);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_robot_left", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_robot_left",      "/base", ros::Time(0), tf_robot_left);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_human_right", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_human_right",     "/base", ros::Time(0), tf_human_right);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_human_center", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_human_center",    "/base", ros::Time(0), tf_human_center);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_human_left", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_human_left",      "/base", ros::Time(0), tf_human_left);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_boundary_right", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_boundary_right",  "/base", ros::Time(0), tf_bound_right);

                listener.waitForTransform("/base", ros::Time(0), "cloth_point_boundary_left", current_time, "/base", ros::Duration(1.0));
                listener.lookupTransform("cloth_point_boundary_left",   "/base", ros::Time(0), tf_bound_left);

                listener.waitForTransform("/base", ros::Time(0), "right_endgripper2", current_time, "/right_gripper", ros::Duration(1.0));
                listener.lookupTransform("/base",  ros::Time(0), "right_endgripper2", current_time, "/right_gripper", tf_gripper_right);

                listener.waitForTransform("/base", ros::Time(0), "left_endgripper2", current_time, "/left_gripper", ros::Duration(1.0));
                listener.lookupTransform("/base",  ros::Time(0), "left_endgripper2", current_time, "/left_gripper", tf_gripper_left);

                // std::cout << "Y" << std::endl;
                // printf("robot : %.2lf \t %.2lf \t %.2lf\n", tf_robot_right.getOrigin().y(), tf_robot_center.getOrigin().y(), tf_robot_left.getOrigin().y());
                // printf("bound : %.2lf \t       \t %.2lf\n", tf_bound_right.getOrigin().y(), tf_bound_left.getOrigin().y());
                // printf("human : %.2lf \t %.2lf \t %.2lf\n", tf_human_left.getOrigin().y(),  tf_human_center.getOrigin().y(), tf_human_right.getOrigin().y());
                // std::cout << "Z" << std::endl;
                // printf("robot : %.2lf \t %.2lf \t %.2lf\n", tf_robot_right.getOrigin().z(), tf_robot_center.getOrigin().z(), tf_robot_left.getOrigin().z());
                // printf("bound : %.2lf \t       \t %.2lf\n", tf_bound_right.getOrigin().z(), tf_bound_left.getOrigin().z());
                // printf("human : %.2lf \t %.2lf \t %.2lf\n", tf_human_left.getOrigin().z(),  tf_human_center.getOrigin().z(), tf_human_right.getOrigin().z());             

                const double error = 0.03;

                hand_poses.header.frame_id = "/base";
                hand_poses.header.stamp = current_time;
                pub_topic.publish(hand_poses);
                if (tf_gripper_right.getOrigin().z() < (double)(CLOTH_WIDTH / 2.0) + 0.01 &&
                    tf_gripper_right.getOrigin().z() > (double)(CLOTH_WIDTH / 2.0) - 0.01 &&
                    tf_gripper_left.getOrigin().z()  < (double)(CLOTH_WIDTH / 2.0) + 0.01 &&
                    tf_gripper_left.getOrigin().z()  > (double)(CLOTH_WIDTH / 2.0) - 0.01 ) {
                    // std::cout << std::abs(P_Rc_x - P_Hc_x) << std::endl;
                    // FOLDING
                    if (P_Hr_z < P_Hc_z - error && P_Hl_z > P_Hc_z + error ) {
                        std::cout << "MOTION : ROTATION (LEFT)" << endl;
                    } else if (P_Hr_z > P_Hc_z + error && P_Hl_z < P_Hc_z - error ) {
                        std::cout << "MOTION : ROTATION (RIGHT)" << endl;
                    } else if (std::abs(P_Rr_y - P_Rl_y) > std::abs(P_Hr_y - P_Hl_y) + error) {
                        std::cout << "MOTION : FOLDING 1" << endl;
                    } else if (std::abs(P_Rc_x - P_Hc_x) < 0.3 || (P_Hr_z < P_Bl_z - 0.01 && P_Rl_z < P_Bl_z - 0.01 && P_Hl_z < P_Br_z - 0.01 && P_Rr_z < P_Br_z - 0.01)) {
                        std::cout << "MOTION : FOLDING 2 (FACE TO FACE)" << endl;
                    } else {
                        std::cout << "MOTION : OUTSTREACH" << endl;
                    }
                     std::cout << endl;
                }
                loop_rate.sleep();
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_node");

    cout << "Initializing node... " << endl;
    Test test;
    test.gen_path();

    return 0;
}
