#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>


void traffic_sign_callback(std_msgs::Int8 msg) {

}

void traffic_light_callback(std_msgs::Int8 msg) {
    
}

void odom_calback(nav_msgs::Odometry msg) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mode_controller_node");

    ros::Subscriber traffic_sign_sub = nh.subscribe("traffic_sign", 1, traffic_sign_callback);
    ros::Subscriber traffic_light_sub = nh.subscribe("traffic_light", 1, traffic_light_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Publisher mode_pub = nh.advertise<race::mode>("mode", 1);

    return 0;
}


/*
mode msg
  status
0 : 정지
1 : 진행
   mode
0 : 기본 주행
1 : 좌회전
2 : 우회전
3 : 동적 장애물
4 : 정적 장애물
5 : 차선변경
6 : 주차
*/