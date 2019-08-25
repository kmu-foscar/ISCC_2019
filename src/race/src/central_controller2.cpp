#include <ros/ros.h>

#include <race/lane_info.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>
#include <race/drive_values.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <vector>
#include <fstream>
#include <cstring>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <mutex>

#define _USE_MATH_DEFINES

struct Point {
    double x;
    double y;
    Point() {x = 0; y = 0;}
    Point(double _x, double _y) : x(_x), y(_y) {}
};

enum { BASE, };


double path_arrived_threshold = 2.0;
double yaw_refresh_threshold = 1.0;


int mode = BASE;
int current_path_index = 0;
std::vector<Point> path;
bool is_path_set = false;
Point current_position;
Point rear_position;
bool is_lane_detected = false;
float yaw = 0.0;
ros::Publisher drive_msg_pub;
Point initial_position;

Point prev_position;

float front_heading = 0.0;
float rear_heading = 0.0;


double cal_distance(const Point A, const Point B) {
    return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

double getAngle(std::vector<Point> v1, std::vector<Point> v2) {
    double x1, y1, x2, y2;
    x1 = v1[1].x - v1[0].x;
    y1 = v1[1].y - v1[0].y;
    x2 = v2[1].x - v2[0].x;
    y2 = v2[1].y - v2[0].y;

    double u1 = sqrt(x1*x1 + y1*y1);
    double u2 = sqrt(x2*x2 + y2*y2);
    x1 /= u1;
    y1 /= u1;
    x2 /= u2;
    y2 /= u2;

    // std::cout << x1 << ' ' << y1 << ' ' << x2 << ' ' << y2 << std::endl;
    return asin((x1*y2+x2*y1)/(cal_distance(v2[0], v2[1]))) * 180.0 / M_PI;

}

void set_path() {
    std::string HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::ifstream infile(HOME+"/ISCC_2019/src/race/src/path6.txt");
    std::string line;

    float min_dis = 9999999;
    float x, y;
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
    initial_position.x = current_position.x;
    initial_position.y = current_position.y;

    ROS_INFO("path initialized, index : %d, position : %f %f", current_path_index, current_position.x, current_position.y);
    
    is_path_set = true;
}


void odom_front_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;
    front_heading = odom->pose.pose.position.z;

    if(!is_path_set) {
        set_path();
    }

    if(mode == BASE && is_path_set) {
        double steering, throttle=5;
        std::vector<Point> v1, v2;

        Point center_point;
        Point temp;
        temp.x = 1*cos(front_heading);
        temp.y = 1*sin(front_heading);

        // steering 계산 부분
        v1.push_back(center_point);
        v1.push_back(temp);
        v2.push_back(current_position);
        v2.push_back(path[current_path_index]);

        std::cout << "current_position : " << current_position.x << ' ' << current_position.y << std::endl;

        steering = getAngle(v1, v2);
        
        // drive msg publising 부분
        race::drive_values drive_msg;
        
        if(steering >= 28) steering = 28;
	    if(steering <= -28) steering = -28;
        
        drive_msg.throttle = (int)throttle;
        drive_msg.steering = (int)(steering*-1);
        
        // ROS_INFO("steering : %f", steering);
        // std::cout << "steering : " << steering << std::endl;

        drive_msg_pub.publish(drive_msg);
    }
    std::cout << current_path_index << ' ' << cal_distance(current_position, rear_position) << std::endl;
    if(cal_distance(current_position, path[current_path_index]) < path_arrived_threshold) current_path_index++;
    if(cal_distance(current_position, prev_position) > yaw_refresh_threshold) {
        prev_position.x = current_position.x;
        prev_position.y = current_position.y;
    }

}

void odom_rear_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    rear_position.x = odom->pose.pose.position.x;
    rear_position.y = odom->pose.pose.position.y; 
    rear_heading = odom->pose.pose.position.z;
    std::cout << "rear_position : " << rear_position.x << ' ' << rear_position.y << std::endl;
}

void lane_info_callback(const race::lane_info::ConstPtr& msg) {

}

void mode_callback(const race::mode::ConstPtr& msg) {
    mode = msg->mode;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "central_controller_node2");
    ros::NodeHandle nh;

    ros::Subscriber odom_front_sub = nh.subscribe("odom_front", 1, odom_front_callback);
    ros::Subscriber odom_rear_sub = nh.subscribe("odom_rear", 1, odom_rear_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    ros::spin();
}
