#include <ros/ros.h>

#include <race/lane_info.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>
#include <race/drive_values.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>

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

enum { BASE_WITHOUT_LANE_DETECTION, BASE_WITH_LANE_DETECTION, STATIC_OBSTACLE_1, STATIC_OBSTACLE_2, DYNAMIC_OBSTACLE};


double path_arrived_threshold = 2.0;

int mode = DYNAMIC_OBSTACLE;
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

float y_clipping_threshold = 2.5;
int obstacle_1_started = 0;

bool dynamic_obstacle_flag = false;
int dynamc_obstacle_cnt = 0;
double steering, throttle=5;

bool mode_changable = true;


float data_transform(float x, float in_min, float in_max, float out_min, float out_max) // 적외선 센서 데이터 변환 함수
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool point_cmp(const Point A, const Point B) {
    if(fabs(A.y - B.y) < 0.3) {
        return fabs(A.x) < fabs(B.x);
    }
    return A.y < B.y;
}

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

    std::cout << "v1 : " << x1 << ' ' << y1 << std::endl;
    std::cout << "v2 : " << x2 << ' ' << y2 << std::endl;

    // std::cout << x1 << ' ' << y1 << ' ' << x2 << ' ' << y2 << std::endl;
    // return asin((x1*y2-x2*y1)/(cal_distance(v2[0], v2[1]))) * 180.0 / M_PI;
    float ang1 = atan2(y1, x1) * 180.0 / M_PI;
    float ang2 = atan2(y2, x2) * 180.0 / M_PI;
    if(ang1 < 0) ang1 += 360;
    if(ang2 < 0) ang2 += 360;

    std::cout << "asin v1, v2 : " << ang1 << ' ' << ang2 << std::endl;

    if(ang1 - ang2 > 180)
    	return (ang1 - ang2) - 360; 
    if(ang1 - ang2 < -180)
    	return 360 + (ang1 - ang2);
    return ang1 - ang2;
}

bool operator<(geometry_msgs::Point A, geometry_msgs::Point B) {
    if(A.x == B.x) {
        return A.y < B.y;
    }
    return A.x < B.x;
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

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    yaw = msg->orientation.z;
}


void odom_front_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;
    front_heading = odom->pose.pose.position.z;

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

    if(mode == BASE_WITHOUT_LANE_DETECTION) {
        std::vector<Point> v1, v2;

        Point center_point; // (0,0)
        Point temp;
        
        temp.x = 1*cos(yaw*M_PI/180.0);
        temp.y = 1*sin(yaw*M_PI/180.0);

	    // std::cout << "current_position : " << current_position.x << ' ' << current_position.y << std::endl;
        std::cout << "yaw : " << yaw << std::endl;
        // steering 계산 부분
        v1.push_back(center_point);
        v1.push_back(temp);
        v2.push_back(current_position);
        v2.push_back(path[current_path_index+2]);
        
        steering = getAngle(v1, v2);
        
        race::drive_values drive_msg;
        
        // throttle = data_transform(-abs(steering), -180, 0, 3, 8);

        drive_msg.throttle = throttle;
        drive_msg.steering = steering;
        
        // ROS_INFO("steering : %f", steering);
        std::cout << "steering : " << drive_msg.steering << std::endl;

        drive_msg_pub.publish(drive_msg);
    } else if(mode == BASE_WITH_LANE_DETECTION) {
        float gps_base_steering, lane_detection_base_steering = 0;

        std::vector<Point> v1, v2;

        Point center_point; // (0,0)
        Point temp;
        
        temp.x = 1*cos(yaw*M_PI/180.0);
        temp.y = 1*sin(yaw*M_PI/180.0);

        // std::cout << "current_position : " << current_position.x << ' ' << current_position.y << std::endl;
        std::cout << "yaw : " << yaw << std::endl;
        // steering 계산 부분
        v1.push_back(center_point);
        v1.push_back(temp);
        v2.push_back(current_position);
        v2.push_back(path[current_path_index+2]);
        
        gps_base_steering = getAngle(v1, v2);

        
    }
    
    std::cout << current_path_index << std::endl;
    if(cal_distance(current_position, path[current_path_index]) < path_arrived_threshold) current_path_index++;
}


void obstacle_callback(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg) {
    race::drive_values drive_msg;
    if(mode == STATIC_OBSTACLE_1) {
        mode_changable = false;
        std::vector<Point> obstacles;
        std::cout << "------------obstacles------------" << std::endl;
        for(int i = 0 ; i < obstacles_msg->circles.size() ; i++) {
            float x_pos = obstacles_msg->circles[i].center.x;
            float y_pos = obstacles_msg->circles[i].center.y;
            if(fabs(y_pos) < y_clipping_threshold) {
                obstacles.push_back(Point(-y_pos, x_pos));
                std::cout << obstacles.back().x << ' ' << obstacles.back().y << std::endl;
            }
        }
        std::cout << "-----------sorted_obstacles----------" << std::endl;
        sort(obstacles.begin(), obstacles.end(), point_cmp);
        if(obstacles.size() <= 0) {
            drive_msg.throttle = 5;
            drive_msg.steering = 0;
            drive_msg_pub.publish(drive_msg);
            return;
        }
        for(int i = 0 ; i < obstacles.size() ; i++) {
            std::cout << obstacles[i].x << ' ' << obstacles[i].y << std::endl;
        }
        if(obstacles[0].x < 0 && obstacle_1_started == 0) {
            obstacle_1_started = 1;
        }
        if(obstacles[0].x >= 0 && obstacle_1_started == 0) {
            obstacle_1_started = 2;
        }
        std::cout << obstacle_1_started << std::endl;
        if(obstacle_1_started == 1) {
            int idx = 0;
            while(obstacles[idx].y < 0.3) idx++;
            if(idx >= obstacles.size()) return;
            steering = 43-((atan2(obstacles[idx].y, obstacles[idx].x)*180.0/M_PI)-90);
            std::cout << obstacle_1_started << ' ' << steering << std::endl;
            if(obstacles[idx].y < 0.5) obstacle_1_started = 3;
        } else if(obstacle_1_started == 2) {
            int idx = 0;
            while(obstacles[idx].y < 0.3) idx++;
            if(idx >= obstacles.size()) return;
            steering = -((atan2(obstacles[idx].y, obstacles[idx].x)*180.0/M_PI)-90+50);
            std::cout << obstacle_1_started << ' ' << steering << std::endl;
            if(obstacles[idx].y < 0.5) obstacle_1_started = 4;
        } else if(obstacle_1_started == 3) {
            int idx = 0;
            while(obstacles[idx].y < 0.3) idx++;
            if(idx >= obstacles.size()) return;
            steering = -((atan2(obstacles[idx].y, obstacles[idx].x)*180.0/M_PI)-90+50);
            std::cout << obstacle_1_started << ' ' << steering << std::endl;
            if(obstacles[idx].y < 0.5 && obstacles[idx].x > 0) obstacle_1_started = 5;
        } else if(obstacle_1_started == 4) {
            int idx = 0;
            while(obstacles[idx].y < 0.3) idx++;
            if(idx >= obstacles.size()) return;
            steering = 40-((atan2(obstacles[idx].y, obstacles[idx].x)*180.0/M_PI)-90);
            std::cout << obstacle_1_started << ' ' << steering << std::endl;
            if(obstacles[idx].y < 0.5 && obstacles[idx].x < 0) obstacle_1_started = 5;
        } else if(obstacle_1_started == 5) {
            throttle = 5;
            steering = 0;
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
            current_path_index = nearest_idx;
            mode = BASE_WITH_LANE_DETECTION;
        }
        drive_msg.throttle = throttle;
        drive_msg.steering = steering;
        drive_msg_pub.publish(drive_msg);
    } else if(mode == DYNAMIC_OBSTACLE) {
        mode_changable = false;
        std::vector<Point> obstacles;
        std::cout << "------------obstacles------------" << std::endl;
        for(int i = 0 ; i < obstacles_msg->circles.size() ; i++) {
            float x_pos = obstacles_msg->circles[i].center.x;
            float y_pos = obstacles_msg->circles[i].center.y;
            if(fabs(y_pos) < 1.0) {
                obstacles.push_back(Point(-y_pos, x_pos));
                std::cout << obstacles.back().x << ' ' << obstacles.back().y << std::endl;
            }
        }
        if(obstacles.size() != 0 && dynamic_obstacle_flag == false) {
            throttle = 0;
            dynamic_obstacle_flag = true;
        }
        if(dynamic_obstacle_flag == true && obstacles.size() == 0) {
        	if(dynamc_obstacle_cnt == 30) {
        		mode = BASE_WITH_LANE_DETECTION;
            	throttle = 5;
            	std::cout << "finish" << std::endl;	
        	} else {
        		dynamc_obstacle_cnt++;
        	}
        } else if(dynamic_obstacle_flag == true && obstacles.size() != 0) {
        	dynamc_obstacle_cnt = 0;
        }
        drive_msg.throttle = throttle;
        drive_msg.steering = steering;
        drive_msg_pub.publish(drive_msg);
    }
}

void lane_info_callback(const race::lane_info::ConstPtr& msg) {

}

void mode_callback(const race::mode::ConstPtr& msg) {
    if(mode_changable == false) return;

    bool gps_flag = false;
    bool lane_detection_flag = false;
    bool static_obstacle_flag = false;
    bool dynamic_obstacle_flag = false;
    bool parking_flag = false;
 
    unsigned int mode_ = mode->pmode;
    if(mode%2 == 1) // Parking
        parking_flag = true;
    mode>>1;
    if(mode%2 == 1) // Dynamic Obstacle
        dynamic_obstacle_flag = true;
    mode>>1;
    if(mode%2 == 1) // Static Obstacle
        static_obstacle_flag = true;
    mode>>1;
    if(mode%2 == 1) // Lane Detector
        lane_detection_flag = true;
    mode>>1;
    if(mode%2 == 1) // GPS
        gps_flag = true;

    if(gps_flag && lane_detection_flag) {
        mode = BASE_WITH_LANE_DETECTION;
    }
    if(gps_flag && !lane_detection_flag) {
        mode = BASE_WITHOUT_LANE_DETECTION;
    }
    if(dynamic_obstacle_flag) {
        mode = DYNAMIC_OBSTACLE;
    }
    if(static_obstacle_flag) {
        mode = STATIC_OBSTACLE_1;
    }

    if(mode->pstatus == 0) throttle = 0;
    else throttle = 5;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "central_controller_node2");
    ros::NodeHandle nh;

    ros::Subscriber odom_front_sub = nh.subscribe("odom_front", 1, odom_front_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imu_callback);
    ros::Subscriber obstacle_sub = nh.subscribe("obstacles", 1, obstacle_callback);
    drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    ros::spin();
}
