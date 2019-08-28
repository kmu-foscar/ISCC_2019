/* 2019-08-28 금요일
 * 20153155 김다훈
 * 20153183 박호준
 * mode_controller.cpp 예선
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include "race/mode.h"

std_msgs::Int8 pstatus;
std_msgs::Int8 pmode;
std_msgs::Int8 pspd_limit;

struct Point {
    float x;
    float y;
    Point() {x = 0; y = 0;}
    Point(float _x, float _y) : x(_x), y(_y) {}
};

Point current_position;
int mission_num = 0; // 예선전 미션 숫자 목록 저장 변수

/* 미션 목록
 * 0. 일반주행
 * 1. 교차로 좌회전
 * 2. 정적 장애물(드럼통)
 * 3. 교차로 우회전
 * 4. 교차로 직진(신호등)
 * 5. 교차로 좌회전
 * 6. 돌발 장애물/일시정지
 * 7. 교차로 좌회전(신호등)
 * 8. 교차로 직진(신호등)
 * 9. 교차로 직진
 * 10.주차
 */




void traffic_sign_callback(std_msgs::UInt8 msg) { // 표지판 메세지 콜백 함수

    std_msgs::UInt8 ts_flag;
    //ts_flag.data = 0b00001111;
    //ts_flag.data = ts_flag.data & msg.data;
    ts_flag.data = 0;
    ts_flag.data = msg.data;

    // 1. 주차 0001 or 1 
    // 2. 어린이보호구역, 제한속도 30     0010 or 2
    // 3, 감속문턱사인   0011 or 3 

    if(ts_flag.data == 1){          //어린이 보호구역 간판 사인         
        pspd_limit.data = 30;
   }
        
    else if(ts_flag.data == 2){          // 주차미션 간판 사인
        mission_num = 10;
   }

    else if(ts_flag.data == 3){          //감속문턱 간판 사인
       //감속 필요에 따라
        pspd_limit.data = 30;
    }
    
    else{
        pspd_limit.data = 50;               //-> 기본 제한 속도
     }


}

/* 청신호 000X
 * 좌회전 00X0
 * 황신호 0X00
 * 적신호 X000
 */
void traffic_light_callback(std_msgs::UInt8 msg) { // 신호등 메세지 콜백 함수
    std_msgs::UInt8 tl_flag;
    tl_flag.data = 0b00001111;
    tl_flag.data = tl_flag.data & msg.data;

    if(tl_flag.data == 1) { //청신호 0001
        
        pstatus.data = 1;

    }


    else if(tl_flag.data == 3) { //청, 좌회전 0011
        
        pstatus.data = 1;
        mission_num = 7;

    }

    else if(tl_flag.data == 10){ //적, 좌회전 1010

        pstatus.data = 1;
        mission_num = 7;

    }

    // 황,적신호 1100
    // 적신호    1000
    // 황신호    0100
    else {
        pstatus.data = 0;
    }
}

void odom_callback(nav_msgs::Odometry msg) { // Odom 메세지 콜백 함수
    // Odom에서 진행중인 미션의 완료를 판단
/* 미션 목록
 * 0. 일반주행
 * 1. 교차로 좌회전
 * 2. 정적 장애물(드럼통)
 * 3. 교차로 우회전
 * 4. 교차로 직진(신호등)
 * 5. 교차로 좌회전
 * 6. 돌발 장애물/일시정지
 * 7. 교차로 좌회전(신호등)
 * 8. 교차로 직진(신호등)
 * 9. 교차로 직진
 * 10.주차
 */
    

    current_position.x = msg.pose.pose.position.x;
    current_position.y = msg.pose.pose.position.y;


/* 미션 */


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mode_controller_node");

    pstatus.data = 1;
    pmode.data = 0;

    ros::NodeHandle nh;
    race::mode m;

    ros::Subscriber traffic_sign_sub = nh.subscribe("traffic_sign", 1, traffic_sign_callback);
    ros::Subscriber traffic_light_sub = nh.subscribe("traffic_light", 1, traffic_light_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);

    ros::Publisher mode_pub = nh.advertise<race::mode>("mode", 1);

    // TODO 표지판, 신호등, 차선 정보 가공
    if(pstatus.data) {
        if(mission_num == 4 || mission_num == 8 || mission_num == 9) {
            m.mode = 0;
        }
        else if(mission_num == 0 || mission_num == 1 || mission_num == 5 || mission_num == 7) {
            m.mode = 1;
        }
        else if(mission_num == 3) {
            m.mode = 2;
        }
        else if(mission_num == 6) {
            m.mode = 3;
        }
        else if(mission_num == 2) {
            m.mode = 4;
        }
        else if(mission_num == 10) {
            m.mode = 6;
        }

    }

    // mode 발행
    m.status = pstatus.data;
    m.spd_limit = pspd_limit.data;
    mode_pub.publish(m);

    ros::spin();
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