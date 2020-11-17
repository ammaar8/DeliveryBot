#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>(
        "joint_states", 1
    );
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    double joint_door = 1, joint_left_wheel = 0, joint_right_wheel = 0, angle=0;
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()){
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] = "joint_door";
        joint_state.position[0] = joint_door;

        joint_state.name[1] = "joint_left_wheel";
        joint_state.position[1] = joint_left_wheel;

        joint_state.name[2] = "joint_right_wheel";
        joint_state.position[2] = joint_right_wheel;
        
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        joint_door += 0.01;
        joint_left_wheel += 0.1;
        joint_right_wheel += 0.1;
        angle -= degree / 4;
        loop_rate.sleep();
    }
    return 0;
}
