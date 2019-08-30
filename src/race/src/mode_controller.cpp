/* 2019-08-30 금요일
 * 20153155 김다훈
 * 20153183 박호준
 * mode_controller.cpp 예선
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include "race/mode.h"



struct Point {
    float x;
    float y;
    Point() {x = 0; y = 0;}
    Point(float _x, float _y) : x(_x), y(_y) {}
};


std_msgs::Int8 pstatus;
std_msgs::Int8 pmode;
Point current_position;
int traffic_light_num;
int traffic_sign_num; //예선전 표지판 숫자 목록 저장 변수
int mission_num; // 예선전 미션 숫자 목록 저장 변수


// 미션목록
enum MISSION{

    BASE,                   //0. 일반주행
    TURN_LEFT,              //1. 교차로 좌회전
    STATIC_OBSTACLE,        //2. 정적 장애물(드럼통)
    TURN_RIGHT,             //3. 교차로 우회전
    GO_STRAIGHT_TRAFFIC_1,  //4. 교차로 직진(신호등) -> 불빛(청)
    TURN_LEFT_TRAFFIC_1,    //5. 교차로 좌회전(신호등) -> 불빛(적,좌회전)
    DYNAMIC_OBSTACLE,       //6. 돌발 장애물/일시정지
    TURN_LEFT_TRAFFIC_2,    //7. 교차로 좌회전(신호등) -> 불빛(초,좌회전)
    GO_STRAIGHT_TRAFFIC_2,  //8. 교차로 직진(신호등) -> 불빛(청, 좌회전)
    GO_STRAIGHT,            //9. 교차로 직진
    //PARKING,                //10 .주차

};

//신호등 신호
enum TRAFFIC_LIGHT{

    LIGHT_NOPE, //아무것도 없음
    LIGHT_RED, // 빨간불
    LIGHT_YELLOW,// 노랑불
    LIGHT_GREEN,// 초록불
    LIGHT_RED_YELLOW, //빨강,노랑불    
    LIGHT_LEFT_RED, //빨강 좌회전
    LIGHT_LEFT_GREEN// 초록 좌회전
};


// 표지판 신호
enum TRAFFIC_SIGN{

    SIGN_NOPE, //아무것도 없음
    SIGN_LEFT, // 좌회전 표지판
    SIGN_RIGHT, //우회전 표지판
    SIGN_STATIC_OBSTACLE, // 공사중(정적 장애물)표지판
    SIGN_DYNAMIC_OBSTACLE, // 자전거(동적 장애물)표지판
    SIGN_FOUR_DISTANCE, // 4거리 표지판
    //PARKING_SIGN, // 주차 표지판 -> 예선전엔 없음

};


void traffic_sign_callback(std_msgs::UInt8 msg) { // 표지판 메세지 콜백 함수

    std_msgs::UInt8 ts_flag;
    ts_flag.data = 0;
    ts_flag.data = msg.data;

    
    if(ts_flag.data == SIGN_LEFT){                       
        traffic_sign_num = SIGN_LEFT; // 첫번째 좌회전 표지판에선 비신호 좌회전
   }
    else if(ts_flag.data == SIGN_RIGHT){ // 우회전 표지판
        traffic_sign_num = SIGN_RIGHT;
    }
    else if(ts_flag.data == SIGN_STATIC_OBSTACLE){ // 공사중(정적장애물)표지판
        traffic_sign_num = SIGN_STATIC_OBSTACLE;
    }
    else if(ts_flag.data == SIGN_DYNAMIC_OBSTACLE){ // 자전거(동적장애물)표지판
        traffic_sign_num = SIGN_DYNAMIC_OBSTACLE;
    }
    else if(ts_flag.data == SIGN_FOUR_DISTANCE){
        traffic_sign_num = SIGN_FOUR_DISTANCE;
    }
    else{
        traffic_sign_num = SIGN_NOPE;
    }
    /*else if(ts_flag.data == PARKING_SIGN){ //주차 표지판 -> 예선전엔 없음
        mission_num = PAR
        KING;
   }*/ 

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
        traffic_light_num = LIGHT_GREEN;

    }

    else if(tl_flag.data == 3) { //청, 좌회전 0011
        
        pstatus.data = 1;
        traffic_light_num = LIGHT_LEFT_GREEN;
    }
    else if(tl_flag.data == 4) { // 황신호 0100
        pstatus.data = 0;
        traffic_light_num = LIGHT_YELLOW;
    }
    else if(tl_flag.data == 8) { // 적신호 1000
        pstatus.data = 0;
        traffic_light_num = LIGHT_RED;
    }

    else if(tl_flag.data == 10){ //적, 좌회전 1010

        pstatus.data = 1;
        traffic_light_num = LIGHT_LEFT_RED;
    }

    else if(tl_flag.data == 12) { // 황,적신호 1100
        pstatus.data = 0;
        traffic_light_num = LIGHT_RED_YELLOW;
    }
    else{
        traffic_light_num = LIGHT_NOPE;
    }
}

void odom_callback(nav_msgs::Odometry msg) { // Odom 메세지 콜백 함수
 /* Odom에서 진행중인 미션의 완료를 판단
    0. 일반주행
    1. 교차로 좌회전
    2. 정적 장애물(드럼통)
    3. 교차로 우회전
    4. 교차로 직진(신호등) -> 불빛(청)
    5. 교차로 좌회전(신호등) -> 불빛(적,좌회전)
    6. 돌발 장애물/일시정지
    7. 교차로 좌회전(신호등) -> 불빛(초,좌회전)
    8. 교차로 직진(신호등) -> 불빛(청, 좌회전)
    9. 교차로 직진
*/        

    current_position.x = msg.pose.pose.position.x;
    current_position.y = msg.pose.pose.position.y;

/* 미션 */

/*
if(msg.data == 미션 1번 좌표 인덱스 범위 && tfaiic_sign_num == SIGN_LEFT){

    mission_num = TURN_LEFT;

}

else if(msg.data == 미션 2번 좌표 인덱스 범위 && traffic_sign_num == SIGN_STATIC_OBSTACLE){

    mission_num = STATIC_OBSTACLE;

}

else if(msg.data == 미션 3번 좌표 인덱스 범위 && traffic_sign_num == SIGN_RIGHT){

    mission_num = TURN_RIGHT;

}

else if(msg.data == 미션 4번 좌표 인덱스 범위 && traffic_light_num == LIGHT_GREEN){

    mission_num = GO_STRAIGHT_TRAFFIC_1;

}
else if(msg.data == 미션 5번 좌표 인덱스 범위 && traffic_light_num == LIGHT_LEFT_RED ){

    mission_num = TURN_LEFT_TRAFFIC_1;

}
else if(msg.data == 미션 6번 좌표 인덱스 범위 && traffic_sign_num == SIGN_DYNAMIC_OBSTACLE){

    mission_num = DYNAMIC_OBSTACLE;

}

else if(msg.data == 미션 7번 좌표 인덱스 범위 && traffic_light_num == LIGHT_LEFT_GREEN ){

    mission_num = TURN_LEFT_TRAFFIC_2;

}

else if(msg.data == 미션 8번 좌표 인덱스 범위 && traffic_light_num == LIGHT_GREEN ){

    mission_num = GO_STRAIGHT_TRAFFIC_2;

}

else if(msg.data == 미션 9번 좌표 인덱스 범위 && traffic_sign_num == SIGN_FOUR_DISTANCE ){

    mission_num = GO_STRAIGHT;

}

else{

    mission_num = BASE;

}

*/

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mode_controller_node");

    pstatus.data = 1;
    pmode.data = 0;
    mission_num = BASE;
    traffic_light_num = LIGHT_NOPE;
    traffic_sign_num = SIGN_NOPE;

    ros::NodeHandle nh;
    race::mode m;

    ros::Subscriber traffic_sign_sub = nh.subscribe("traffic_sign", 1, traffic_sign_callback);
    ros::Subscriber traffic_light_sub = nh.subscribe("traffic_light", 1, traffic_light_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Publisher mode_pub = nh.advertise<race::mode>("mode", 1);


    // TODO 표지판, 신호등, 차선 정보 가공
    if(pstatus.data) {
        if(mission_num == BASE|| mission_num == GO_STRAIGHT_TRAFFIC_1 || mission_num == GO_STRAIGHT) {
            m.mode = 0; // 일반주행
        } 
        else if(mission_num == TURN_LEFT || mission_num == TURN_LEFT_TRAFFIC_1 || mission_num == TURN_LEFT_TRAFFIC_2) {
            m.mode = 1; // 좌회전
        }
        else if(mission_num == TURN_RIGHT) {
            m.mode = 2; // 우회전
        }
        else if(mission_num == DYNAMIC_OBSTACLE) {
            m.mode = 3; // 동적장애물
        }
        else if(mission_num == STATIC_OBSTACLE) {
            m.mode = 4; // 정적 장애물
        }
        /*else if(mission_num == PARKING) {
            m.mode = 6; // 주차--> 예선전엔 없음
        }*/

    }

    // mode 발행
    m.status = pstatus.data;
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
