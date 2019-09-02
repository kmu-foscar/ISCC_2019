// 예선

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include "race/mode.h"

// TRAFFIC_LIGHT
#define RED_AND_RIGHT    10     // 1010
#define GREEN_AND_RIGHT  3      // 0011
#define GREEN            1      // 0001

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

uint8_t pmode = 0;
uint8_t traffic_light = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ALL_OFF() { pmode = 0; }

void GPS_DRIVE_ON() { pmode += 16; }
void GPS_DRIVE_OFF() { pmode -= 16; }

void LANE_DETECT_ON() { pmode += 8; }
void LANE_DETECT_OFF() { pmode -= 8; }

void STATIC_OBSTACLE_ON() { pmode += 4; }
void STATIC_OBSTACLE_OFF() { pmode -= 4; }

void DYNAMIC_OBSTACLE_ON() { pmode += 2; }
void DYNAMIC_OBSTACLE_OFF() { pmode -= 2; }

void PARKING_ON() { pmode += 1; }
void PARKING_OFF() { pmode -= 1; }

/*
void STOPLINE_ON() { pmode += 32; }
void STOPLINE_OFF() { pmode -= 32; }

void TRAFFIC_SIGN_ON() { pmode += 16; }
void TRAFFIC_SIGN_OFF() { pmode -= 16; }

void TRAFFIC_LIGHT_ON() { pmode += 8; }
void TRAFFIC_LIGHT_OFF() { pmode -= 8; }
*/

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

    if(gps_point_index < 211) {
        // 
        GPS_DRIVE_ON();
        LANE_DETECT_ON();
    }
    else if(211 =< gps_point_index && gps_point_index < 276) {
        LANE_DETECT_OFF();
    }
    else if(276 =< gps_point_index && gps_point_index < 300) {
        LANE_DETECT_ON();
    }
    else if(300 =< gps_point_index && gps_point_index < 340) {
        ALL_OFF();
        STATIC_OBSTACLE_ON();
    }
    else if(340 =< gps_point_index && gps_point_index < 460) {
        ALL_OFF();
        GPS_DRIVE_ON();
        LANE_DETECT_ON();
    }
    else if(460 =< gps_point_index && gps_point_index < 514) {
        LANE_DETECT_OFF();
    }
    else if(514 =< gps_point_index && gps_point_index < 550) {
        LANE_DETECT_ON();
        if(is_stopline) {
            
        }
    }
    else if(550 =< gps_point_index && gps_point_index < 568) {
        ALL_OFF();
        GPS_DRIVE_ON();
    }
    else if(568 =< gps_point_index && gps_point_index < 662) {
        // 어린이 보호구역
        
    }
    else if(662 =< gps_point_index && gps_point_index < 701) {
        // 어린이 보호구역

    }
    else if(701 =< gps_point_index && gps_point_index < 764) {
        // 어린이 보호구역
    }
    else if(764 =< gps_point_index && gps_point_index < 787) {
    }
    else if(787 =< gps_point_index && gps_point_index < 831) {
    }
    else if(831 =< gps_point_index && gps_point_index < 913) {
    }
    else if(913 =< gps_point_index && gps_point_index < 945) {
    }
    else if(945 =< gps_point_index && gps_point_index < 1015) {
    }
    else if(1015 =< gps_point_index && gps_point_index < 1046) {
    }
    else if(1046 =< gps_point_index && gps_point_index < 1063) {
    }
    else if(1063 =< gps_point_index && gps_point_index < 1161) {
    }
    else if(1161 =< gps_point_index && gps_point_index < 1218) {
    }
    else if(1218 =< gps_point_index && gps_point_index < 1285) {
    }
    else if(1285 =< gps_point_index && gps_point_index < 1514) {
    }
    else {}

    // mode 발행
    m.status;
    m.mode;
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
