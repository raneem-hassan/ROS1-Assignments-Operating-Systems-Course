#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdlib>
#include <ctime>

float getFakePotValue() {
  return rand() % 1024;
}

void altitudeCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  float z = msg->data[0];
  float tolerance = msg->data[1];

  if (z < tolerance && z > -tolerance) {
    ROS_INFO("LED: Continuous flashing (fast)");
  } else {
    ROS_INFO("LED: Flash ON then OFF (slow flash)");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "POT");
  ros::NodeHandle nh;

  srand(time(0)); 

  ros::Publisher pot_pub = nh.advertise<std_msgs::Float32>("pot", 10);
  ros::Subscriber sub = nh.subscribe("altitude", 10, altitudeCallback);

  ros::Rate rate(10);  

  while (ros::ok()) {
    std_msgs::Float32 msg;
    msg.data = getFakePotValue();
    pot_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
