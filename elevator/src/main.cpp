#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <vector>

gazebo::transport::Node node;

void elevatorCB(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    gazebo::msgs::GzString message;
    message.set_data(msg->data.c_str());
    gazebo_pub->Publish(message);
}

int main(int _argc, char **_argv){

    // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    node->Init();
    ros::init(_argc, _argv, "elevator");

    // Create Gazebo node and init


    // Create ROS node and init
    ros::NodeHandle n;
    ros::Subscriber ros_sub = n.subscribe("/floor", 1000, elevatorCB);
    
    gazebo::transport::Publisher gazebo_pub = node->Advertise<gazebo::msgs::GzString>("/gazebo/default/elevator", 1);

    while (true)
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();


    // Mayke sure to shut everything down.

    }
    gazebo::client::shutdown();
}