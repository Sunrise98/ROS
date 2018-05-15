// ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// image.msgから生成されたメッセージを定義しているヘッダ
#include "msg_test/test.h"

#include <stdlib.h>
#include <iostream>
using namespace std;

msg_test::test msg;

int main(int argc, char** argv)
{
	// 初期化宣言
	// このノードは"publisher"という名前であるという意味
	ros::init(argc, argv, "publisher");
	// ノードハンドラの宣言
	ros::NodeHandle n;
	// Publisherとしての定義
	// n.advertise<image_tutorial::image>("image_data", 1000);
	// image_tutorial::image型のメッセージをimage_dataというトピックへ配信する
	// "1000"はトピックキューの最大値
	ros::Publisher pub = n.advertise<msg_test::test>("test_data", 1);
	//1秒間に1回の間隔でループする
	ros::Rate loop_rate(0.5);

	int id = 0;
	// ノードが実行中は基本的にros::ok() = 1
	// Ctrl+Cなどのインタラプトが起こるとros::ok() = 0となる
	while (ros::ok()) {
		msg.ID = id;
		msg.name = "hello";
		// Publishする関数
		pub.publish(msg);
		cout << "published !" << endl;
		ros::spinOnce();
		id++;

		loop_rate.sleep();
	}
	return 0;
}