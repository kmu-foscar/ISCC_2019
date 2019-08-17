#include <ros/ros.h>
#include <race/lane_info.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>
#include <race/drive_values.h>

bool is_lane_detected = false;

Point path[1000];

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	if(is_lane_detected) {

	}

	vec = path[index] - Point(msg->pose.x, msg->pose.y)

	

}

void lane_info_callback(const race::lane_info::ConstPtr& msg) {

}

void mode_callback(const race::mode::ConstPtr& msg) {

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "central_controller_node");
    // To Do 초기 Odometry 설정

    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    ros::Publisher drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    ros::spin();
}