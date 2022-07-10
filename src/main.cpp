#include <iostream>
#include <string>
#include <exception>

#include <ros/ros.h>

#include "util/signal_handler_linux.h"
#include "node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vo_node", ros::init_options::NoSigintHandler);
    SignalHandle::initSignalHandler();

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("vo_node - starts.");
   
    try {  
        std::unique_ptr<MonoNode> node;
        node = std::make_unique<MonoNode>(nh);
    }
    catch (std::exception& e) {
        ROS_ERROR(e.what());
    }
   
    ROS_INFO_STREAM("vo_node - TERMINATED.");
    return -1;
}