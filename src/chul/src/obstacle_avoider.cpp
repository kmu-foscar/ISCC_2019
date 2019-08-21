#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>


#define MIN_SCAN_ANGLE_RAD  -70.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD  +70.0/180*M_PI

using namespace std;

ros::Publisher control_pub;
ros::Subscriber obstacle_sub;

float dist;

bool isdetected = false;

void obstacleCB(const sensor_msgs::LaserScan::ConstPtr& laser) {

    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - laser->angle_min) / laser->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - laser->angle_min) / laser->angle_increment);
	int midIndex = (minIndex + maxIndex) / 2;

	float closestRange_left = laser->ranges[minIndex];
	float closestRange_right = laser->ranges[midIndex];

	for(int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++){
		if(laser->ranges[currIndex] < dist){
			isdetected = true;
            break;
		}
	}



}

int main(int argc, char** argv) {
    ros::init(argc,argv,"dynamic_obstacle");
    ros::NodeHandle nh;

    float dist = (float)atoi(argv[1]);

    obstacle_sub = nh.subscribe("/scan", 10, obstacleCB);

    ros::spin();

}