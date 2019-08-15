#include <ros/ros.h>
#include <race/lane_info.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>
#include <race/drive_values.h>


void odom_callback(nav_msgs::Odometry msg) {

}

void lane_info_callback(race::lane_info msg) {

}

void mode_callback(race::mode msg) {

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "central_controller_node");

    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    ros::Publisher drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    ros::spin();
}