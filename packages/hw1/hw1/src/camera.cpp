#include <iostream>

#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include "hw1/object_position.h"


using namespace std;
using namespace ros;

int main (int argc, char **argv){

    int freq = 100;
    int max_size = 1000;

    if (argc >= 2){
        freq = atoi(argv[1]);
        if (freq > 50) freq = 50;
    }
    else if (argc == 3){
        max_size = atoi(argv[2]);
        if (max_size > 100) max_size = 100;
    }

    init(argc,argv,"Camera");

    NodeHandle n;

    Publisher pub = n.advertise<hw1::object_position> ("ObjectPose", max_size);
    cout << "Rate of Publishing = "<< freq << " and Maximum Number of Messages = "<< max_size<<endl;

    Rate rate(freq);

    srand(static_cast<unsigned int>(time(0)));  

    while (ok()){
        hw1::object_position pos;

        pos.xO = static_cast<float>(rand()) / RAND_MAX * 100.0f;
        pos.yO = static_cast<float>(rand()) / RAND_MAX * 100.0f;

        pub.publish(pos);
        rate.sleep();
    }
 
    return 0;
}