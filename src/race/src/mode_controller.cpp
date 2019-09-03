// 본선

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include "race/mode.h"

// TRAFFIC_LIGHT
#define TL_RED_AND_RIGHT    10     // 1010
#define TL_GREEN_AND_RIGHT  3      // 0011
#define TL_GREEN            1      // 0001

// 청신호 000X
// 좌회전 00X0
// 황신호 0X00
// 적신호 X000

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool gps_drive_flag = false;
bool lane_detect_flag = false;
bool static_obstacle_flag = false;
bool dynamic_obstacle_flag = false;
bool parking_flag = false;

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

void odom_callback(std_msgs::UInt8 msg);
void traffic_light_callback(std_msgs::UInt8 msg);
void traffic_sign_callback(std_msgs::UInt8 msg);
void stopline_callback(std_msgs::UInt8 msg);

int main(void) {

    ros::init(argc, argv, "mode_controller_node");

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

    if(gps_point_index < ) {
        // 시작
    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }
    else if( =< gps_point_index && gps_point_index < ) {

    }

    CALCULATE_MODE_FLAG();

    // mode 발행
    m.status = pstatus;
    m.mode = pmode;

    mode_pub.publish(m);

    ros::spin();
    return 0;
}

void odom_callback(std_msgs::UInt8 msg) {
    gps_point_index = msg.data;
}

void stopline_callback(std_msgs::UInt8 msg) {
    is_stopline = msg.data;
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
