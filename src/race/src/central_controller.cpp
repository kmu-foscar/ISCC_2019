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
#include <cmath>
#include <cstdlib>



struct Point {
    double x;
    double y;
    Point() {x = 0; y = 0;}
    Point(double _x, double _y) : x(_x), y(_y) {}
};

enum { BASE, };

int mode = BASE;
int current_path_index = 0;
std::vector<Point> path;
bool is_path_set = false;
Point current_position;
bool is_lane_detected = false;
double yaw = 0.0;

float cal_distance(const Point A, const Point B) {
    return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

void set_path() {
    std::string HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";


    std::ifstream infile(HOME+"/ISCC_2019/src/race/src/path.txt");
    std::string line;

    double min_dis = 9999999;
    double x, y;
    while(infile >> x >> y) {
        path.push_back(Point(x, y));
        std::cout.precision(11);
	std::cout << std::fixed << path.back().x << ' ' << path.back().y << std::endl;
	double cur_dis = cal_distance(path.back(), current_position);
        if(min_dis > cur_dis) {
	    min_dis = cur_dis;
            current_path_index = path.size()-1;
        }

    }
    ROS_INFO("path initialized, index : %d, position : %f %f", current_path_index, current_position.x, current_position.y);
    is_path_set = true;
}


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;

    if(!is_path_set) {
        set_path();
    }

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

    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu/yaw", 1, imu_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    ros::Publisher drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    // To Do 초기 Odometry 설정


    ros::spin();
}
