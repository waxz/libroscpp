#include <iostream>

#include "ros/ros.h"

#include <thread>
int main(int argc, char** argv) {

    ros::init(argc,argv,"test_node");

    ros::NodeHandle n;

    auto r = ros::Rate(1);
    std::cout << "ros start loop" << std::endl;

    while (true){
        r.sleep();
    }

    return 0;
}
