#include "ros/ros.h"
#include <stdio.h>
#include "msg_test/test.h"
#include <iostream>
using namespace std;

// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void chatterCallback(const msg_test::test msg)
{
  cout << "ID = " << msg.ID <<
          " name = " << msg.name << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  // Subscriberとしてimage_dataというトピックに対してSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = n.subscribe("test_data", 100, chatterCallback);

  // トピック更新の待ちうけを行う関数
  ros::spin();

  return 0;
}