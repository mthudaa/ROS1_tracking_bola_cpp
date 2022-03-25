#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

void callback(const geometry_msgs::Point::ConstPtr& m){
    ROS_INFO("x = %f\n", m->x);
    ROS_INFO("y = %f\n", m->y);
    ROS_INFO("sudut = %f\n", m->z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle n;
    ros::Subscriber xyz_sub = n.subscribe<geometry_msgs::Point>("xyz", 10, callback);
    ros::spin();
    return 0;
}
