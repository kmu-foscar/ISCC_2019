#include <ros/ros.h>
#include <race/lane_info.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>
#include <race/drive_values.h>
#include <vector>
#include <fstream>
#include <cstring>
#include <iostream>
#include <std_msgs/Float64.h>


struct Point {
    int x;
    int y;
    Point() {}
    Point(int _x, int _y) : x(_x), y(_y) {}
};

enum { BASE, };

int mode = BASE;
int path_index = 0;
std::vector<int> a;
std::vector<Point> path;
bool is_path_set = false;
Point current_position;
bool is_lane_detected = false;
float yaw = 0.0;

void set_path() {
    std::ifstream infile("path.txt");
    std::string line;

    while(std::getline(infile, line)) {
        std::istringstream iss(line);
        int x, y;
        if(!(iss >> x >> y)) break;

    }
    ROS_INFO("path is set");
    is_path_set = true;
}


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;

    if(mode == BASE && is_path_set) {

    }

}

void imu_callback(const std_msgs::Float64::ConstPtr& msg) {
    yaw = msg->data;
}

void lane_info_callback(const race::lane_info::ConstPtr& msg) {

}

void mode_callback(const race::mode::ConstPtr& msg) {
    mode = msg->mode;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "central_controller_node");
    ros::NodeHandle nh;

    // To Do 초기 Odometry 설정
    set_path();


    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu/yaw", 1, imu_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    ros::Publisher drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    ros::spin();
}