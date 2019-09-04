// 본선

#include <ros/ros.h>
#include <std_msgs/Int8.h>
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
bool is_stopline = 0;           // 0: 정지선 없음, 1: 정지선 인식

uint8_t pstatus = 0;
uint8_t pmode = 0;

uint8_t traffic_light = 0;
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

/*
void STOPLINE_ON() { pmode += 32; }
void STOPLINE_OFF() { pmode -= 32; }

void TRAFFIC_SIGN_ON() { pmode += 16; }
void TRAFFIC_SIGN_OFF() { pmode -= 16; }

void TRAFFIC_LIGHT_ON() { pmode += 8; }
void TRAFFIC_LIGHT_OFF() { pmode -= 8; }
*/

double cal_distance(const Point A, const Point B) {
    return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

void set_path() {
    std::string HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::ifstream infile(HOME+"/ISCC_2019/src/race/src/path/final_path2.txt");
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
void traffic_light_callback(std_msgs::Int8 msg);
void traffic_sign_callback(std_msgs::Int8 msg);
void stopline_callback(std_msgs::Int8 msg);

int main(int argc, char** argv) {

    ros::init(argc, argv, "mode_controller_node_2");

    ros::NodeHandle nh;
    ros::Subscriber traffic_sign_sub = nh.subscribe("traffic_sign", 1, traffic_sign_callback);
    ros::Subscriber traffic_light_sub = nh.subscribe("traffic_light", 1, traffic_light_callback);
    ros::Subscriber stop_line_sub = nh.subscribe("stopline", 1, stopline_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);

    ros::Publisher mode_pub = nh.advertise<race::mode>("mode", 1);

    race::mode m;
/*
   +-----------+---------------+-----+
   | BIN       | STATEMENT     | DEC |
   +-----------+---------------+-----+
   | 000X 0000 | GPS 자율주행  | 16  |
   | 0000 X000 | 차선인식      | 8   |
   | 0000 0X00 | 정적장애물    | 4   |
   | 0000 00X0 | 동적장애물    | 2   |
   | 0000 000X | 주차          | 1   |
   +-----------+---------------+-----+
*/
    CALCULATE_MODE_FLAG();

    // mode 발행
    m.status = pstatus;
    m.mode = pmode;

    mode_pub.publish(m);

    ros::spin();
    return 0;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;

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

    if(!is_path_set) {
        set_path();
        return;
    }

    gps_point_index = nearest_idx;

    if(gps_point_index < 117) {
        GPS_DRIVE_ON();
        LANE_DETECT_ON();
    }
    else if(117 <= gps_point_index && gps_point_index < 218) {
        // PARKING_ON();
    }
    else if(218 <= gps_point_index && gps_point_index < 273) {
        LANE_DETECT_OFF();
    }
    else if(273 <= gps_point_index && gps_point_index < 454) {
        LANE_DETECT_ON();
    }
    else if(454 <= gps_point_index && gps_point_index < 508) {      // 교차로 우회전
        LANE_DETECT_OFF();
    }
    else if(508 <= gps_point_index && gps_point_index < 536) {
        LANE_DETECT_ON();
    }
    else if(536 <= gps_point_index && gps_point_index < 558) {      // 좌회전 교차로 전
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(558 <= gps_point_index && gps_point_index < 647) {      // 교차로 좌회전
        // GPS_DRIVE_ONLY
    }
    else if(647 <= gps_point_index && gps_point_index < 677) {
        LANE_DETECT_ON();
    }
    else if(677 <= gps_point_index && gps_point_index < 691) {      // 직진 교차로 전
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(691 <= gps_point_index && gps_point_index < 743) {
        // GPS_DRIVE_ONLY
    }
    else if(743 <= gps_point_index && gps_point_index < 882) {
        ALL_OFF();
        LANE_DETECT_ON();
        GPS_DRIVE_ON();
    }
    else if(882 <= gps_point_index && gps_point_index < 952) {      // 정적 장애물(대형버스)
        ALL_OFF();
        GPS_DRIVE_ON();
        STATIC_OBSTACLE_ON();
    }
    else if(952 <= gps_point_index && gps_point_index < 988) {
        ALL_OFF();
        GPS_DRIVE_ON();
        LANE_DETECT_ON();
    }
    else if(988 <= gps_point_index && gps_point_index < 1004) {     // 직진 교차로 전
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(1004 <= gps_point_index && gps_point_index < 1059) {    // 교차로 진입
        // GPS_DRIVE_ONLY
    }
    else if(1059 <= gps_point_index && gps_point_index < 1126) {    // 교차로 탈출
        ALL_OFF();
        GPS_DRIVE_ON();
        LANE_DETECT_ON();
    }
    else if(1126 <= gps_point_index && gps_point_index < 1203) {
        LANE_DETECT_OFF();
    }
    else if(1203 <= gps_point_index && gps_point_index < 1232) {
        LANE_DETECT_ON();
    }
    else if(1232 <= gps_point_index && gps_point_index < 1251) {    // 좌회전 교차로 전
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_LEFT) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(1251 <= gps_point_index && gps_point_index < 1387) {    // 교차로 진입
        // GPS_DRIVE_ONLY
    }
    else if(1387 <= gps_point_index && gps_point_index < 1423) {    // 교차로 탈출
        ALL_OFF();
        GPS_DRIVE_ON();
        LANE_DETECT_ON();
    }
    else if(1423 <= gps_point_index && gps_point_index < 1441) {    // 좌회전 교차로 전
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_LEFT) {                                 // 좌회전 신호에 맞추어 출발
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(1441 <= gps_point_index && gps_point_index < 1507) {
        // GPS_DRIVE_ONLY
    }
    else if(1507 <= gps_point_index && gps_point_index < 1828) {
        LANE_DETECT_ON();
    }
    else if(1828 <= gps_point_index && gps_point_index < 1841) {
        LANE_DETECT_OFF();
    }
    else if(1841 <= gps_point_index && gps_point_index < 1909) {    // 우회전 교차로

    }
    else if(1909 <= gps_point_index && gps_point_index < 2140) {
        LANE_DETECT_ON();
    }
    else if(2140 <= gps_point_index && gps_point_index < 2160) {    // 직진 교차로 전
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(2160 <= gps_point_index && gps_point_index < 2227) {    // 교차로(직진)
        // GPS_DRIVE_ONLY
    }
    else if(2227 <= gps_point_index && gps_point_index < 2258) {
        LANE_DETECT_ON();
    }
    else if(2258 <= gps_point_index && gps_point_index < 2269) {
        if(tl_msg = TL_STOP && is_stopline) {                       // 적신호일때 정지선에 멈춘다.
            pstatus = 0;
            ALL_OFF();
        }
        else if(tl_msg = TL_GANG) {
            pstatus = 1;
            GPS_DRIVE_ON();
        }
    }
    else if(2269 <= gps_point_index && gps_point_index < 2365) {    // 교차로
        // GPS_DRIVE_ONLY
    }
    else if(2365 <= gps_point_index && gps_point_index < 2411) {
        // GPS_DRIVE_ONLY
    }
    else if(2411 <= gps_point_index && gps_point_index < 2465) {
        // GPS_DRIVE_ONLY
    }
    else if(2465 <= gps_point_index) {
        // GPS_DRIVE_ONLY
    }
}

void stopline_callback(std_msgs::Int8 msg) {
    is_stopline = msg.data;
}

void traffic_sign_callback(std_msgs::Int8 msg) {

}

void traffic_light_callback(std_msgs::Int8 msg) {
    tl_msg = msg.data;
}

/*
mode msg

  status
0 : 정지
1 : 진행

  mode
X000 0000 : GPS 자율주행    128
0X00 0000 : 차선인식        64
00X0 0000 : 정지선          32
000X 0000 : 신호등          16
0000 X000 : 표지판          8
0000 0X00 : 정적장애물      4
0000 00X0 : 동적장애물      2
0000 000X : 주차            1
*/
