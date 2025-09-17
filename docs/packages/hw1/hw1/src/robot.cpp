#include <iostream>

#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include "hw1/robot_position.h"
#include "hw1/velocities.h"

using namespace std;
using namespace ros;

void get_velocities (const hw1::velocities::ConstPtr& vel){
    float v = vel->Forward_Vel;
    float w = vel->Forward_Vel;
    ROS_INFO("Subscribing to Velocities topic as : [%f, %f]",v,w);
}
   
int main (int argc, char **argv){

    int freq = 50;
    int max_size = 100;

    init(argc,argv,"Robot");

    NodeHandle n;

    Publisher pub = n.advertise<hw1::robot_position> ("Position", max_size);
    Subscriber sub = n.subscribe("Velocities", max_size, get_velocities);

    Rate rate(freq);

    srand(static_cast<unsigned int>(time(0)));  

    while (ok()){
        hw1::robot_position pos;

        pos.xR = static_cast<float>(rand()) / RAND_MAX * 100.0f;
        pos.yR = static_cast<float>(rand()) / RAND_MAX * 100.0f;
        pos.thR = static_cast<float>(rand()) / RAND_MAX * 3.14f;
        
        pub.publish(pos);
        spinOnce();
        rate.sleep();
    }
 
    return 0;
}