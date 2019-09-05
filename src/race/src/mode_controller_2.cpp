// 본선

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include "race/mode.h"
#include <cstdlib>
#include <fstream>
#include <cstring>
#include <iostream>

// TRAFFIC_LIGHT
//
// 0 : 정지(적신호)
// 1 : 좌회전
// 2 : 신호정보없음(비신호)

#define TL_STOP 0
#define TL_LEFT 1
#define TL_GANG 2

uint8_t tl_msg = 0;

struct Point {
    float x;
    float y;
    Point() {x = 0; y = 0;}
    Point(float _x, float _y) : x(_x), y(_y) {}
};

int gps_point_index = 0;
bool is_stopline = false;           // 0: 정지선 없음, 1: 정지선 인식

uint8_t pstatus = 0;
uint8_t pmode = 0;

std::vector<Point> path;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool gps_drive_flag = false;
bool lane_detect_flag = false;
bool static_obstacle_flag = false;
bool dynamic_obstacle_flag = false;
bool parking_flag = false;

Point current_position;
bool is_path_set = false;
int current_path_index = 0;
Point initial_position;

bool is_parked = false;

void GPS_DRIVE_ON() { gps_drive_flag = true; }
void GPS_DRIVE_OFF() { gps_drive_flag = false; }

void LANE_DETECT_ON() { lane_detect_flag = true; }
void LANE_DETECT_OFF() { lane_detect_flag = false; }

void STATIC_OBSTACLE_ON() { static_obstacle_flag = true; }
void STATIC_OBSTACLE_OFF() { static_obstacle_flag = false; }

void DYNAMIC_OBSTACLE_ON() { dynamic_obstacle_flag = true; }
void DYNAMIC_OBSTACLE_OFF() { dynamic_obstacle_flag = false; }

void PARKING_ON() { parking_flag = true; }
void PARKING_OFF() { parking_flag = false; }

double cal_distance(const Point A, const Point B) {
    return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

void set_path() {
    std::string HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::ifstream infile(HOME+"/ISCC_2019/src/race/src/path/final_path_real_final.txt");
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

void ALL_OFF() {
    gps_drive_flag = false;
    lane_detect_flag = false;
    static_obstacle_flag = false;
    dynamic_obstacle_flag = false;
    parking_flag = false;
}

void CALCULATE_MODE_FLAG() {

    pmode = 0;

    if(gps_drive_flag) { pmode += 16; }
    if(lane_detect_flag) { pmode += 8; }
    if(static_obstacle_flag) { pmode += 4; }
    if(dynamic_obstacle_flag) { pmode += 2; }
    if(parking_flag) { pmode += 1; }
    std::cout << "pmode : " << (int)pmode << std::endl;
    std::cout << "pstatus : " << (int)pstatus << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
void traffic_light_callback(std_msgs::UInt8 msg);
void traffic_sign_callback(std_msgs::UInt8 msg);
void stopline_callback(std_msgs::UInt8 msg);

ros::Publisher mode_pub;

int main(int argc, char** argv) {

    ros::init(argc, argv, "mode_controller_node_2");

    ros::NodeHandle nh;
    mode_pub = nh.advertise<race::mode>("mode", 1);

    ros::Subscriber traffic_sign_sub = nh.subscribe("traffic_sign", 1, traffic_sign_callback);
    ros::Subscriber traffic_light_sub = nh.subscribe("traffic_light", 1, traffic_light_callback);
    ros::Subscriber stop_line_sub = nh.subscribe("stopline", 1, stopline_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom_front", 1, odom_callback);

    ros::spin();
    return 0;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;

    if(!is_path_set) {
        set_path();
        return;
    }

    double minimum_dist = 9999999;
    int nearest_idx = -1;
    for(int i = 0 ; i < path.size() ; i++) {
        double cur_dist_ = cal_distance(current_position, path[i]);
        if(cur_dist_ < minimum_dist) {
            nearest_idx = i;
            minimum_dist = cur_dist_;
        }
    }
    std::cout << "nearest_idx : " << nearest_idx << std::endl;

    gps_point_index = nearest_idx;

    if(gps_point_index < 70) {
        GPS_DRIVE_ON();
    }
    else if(gps_point_index == 70) {
        if(!is_parked) {
            GPS_DRIVE_OFF();
            PARKING_ON();
            is_parked = true;
        } else {
            PARKING_OFF();
            GPS_DRIVE_ON();
        }  
    }
    else if(71 <= gps_point_index && gps_point_index < 228) {
        PARKING_OFF();
        GPS_DRIVE_ON();
    }
    else if(228 <= gps_point_index && gps_point_index < 237) {      
        if(tl_msg == TL_STOP && is_stopline == true) { 
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_LEFT) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(237 <= gps_point_index && gps_point_index < 275) { 
        GPS_DRIVE_ON();
    }
    else if(275 <= gps_point_index && gps_point_index < 282) {   
        if(tl_msg == TL_STOP && is_stopline == true) {                     
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(282 <= gps_point_index && gps_point_index < 397) {
        // GPS_DRIVE_ONLY
    }
    else if(397 <= gps_point_index && gps_point_index < 407) {
        if(tl_msg == TL_STOP && is_stopline == true) {                     
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(407 <= gps_point_index && gps_point_index < 484) {
        // GPS_DRIVE_ONLY
    }
    else if(484 <= gps_point_index && gps_point_index < 493) {  
        if(tl_msg == TL_STOP && is_stopline == true) {                    
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_LEFT) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(493 <= gps_point_index && gps_point_index < 555) {    
        // GPS_DRIVE_ONLY
    }
    else if(555 <= gps_point_index && gps_point_index < 565) {   
        if(tl_msg == TL_STOP && is_stopline == true) {                     
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_LEFT) {                                
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(565 <= gps_point_index && gps_point_index < 790) {
        // GPS_DRIVE_ONLY
    }
    else if(790 <= gps_point_index && gps_point_index < 800) {
        if(tl_msg == TL_STOP && is_stopline == true) {    
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(800 <= gps_point_index && gps_point_index < 830) {    // 교차로(직진)
        // GPS_DRIVE_ONLY
    }
    else if(830 <= gps_point_index && gps_point_index < 835) {
        if(tl_msg == TL_STOP && is_stopline == true) {               
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg == TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(835 <= gps_point_index && gps_point_index < 961) {
        // GPS_DRIVE_ONLY
        GPS_DRIVE_ON();
    }
    else if(961 <= gps_point_index) {
        // GPS_DRIVE_ONLY
        GPS_DRIVE_ON();
    }

    race::mode m;

    CALCULATE_MODE_FLAG();

    // mode 발행
    m.status = pstatus;
    m.mode = pmode;

    mode_pub.publish(m);

}

void stopline_callback(std_msgs::UInt8 msg) {
    if(msg.data == 0)
        is_stopline = false;
    if(msg.data == 1)
        is_stopline = true;
}

void traffic_sign_callback(std_msgs::UInt8 msg) {

}

void traffic_light_callback(std_msgs::UInt8 msg) {
    tl_msg = msg.data;
}

/*
mode msg

  status
0 : 정지
1 : 진행

  mode

     +-----------+---------------+-----+
     | BIN       | STATEMENT     | DEC |
     +-----------+---------------+-----+
     | 000X 0000 | GPS 자율주행    | 16  |
     | 0000 X000 | 차선인식        | 8   |
     | 0000 0X00 | 정적장애물      | 4   |
     | 0000 00X0 | 동적장애물      | 2   |
     | 0000 000X | 주차           | 1   |
     +-----------+---------------+-----+

*/
