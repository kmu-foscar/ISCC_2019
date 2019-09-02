#include <ros/ros.h>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <cmath>

// struct Point {
// 	double x;
// 	double y;
// 	Point() {x = 0; y = 0;}
// 	Point(double _x, double _y) : x(_x), y(_y) {}
// };


// float steering = 0.0;

// double cal_distance(const Point A, const Point B) {
// 	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
// }

// float getAngle(float x1, float y1, float x2, float y2) {
// 	float dotproduct = (x1 * x2) + (y1 * y2);
// 	float size_1 = sqrt((x1 * x1) + (y1 * y1));
// 	float size_2 = sqrt((x2 * x2) + (y2 * y2));

// 	float theta = acos(dotproduct / (size_1 * size_2));

// 	return theta < (M_PI - theta) ? theta : (M_PI - theta);
// }


void obstacle_callback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
// 	Point car_point;
// 	Point obstacle_point;
// 	Point final_point;


// 	car_point.x = 0.0, car_point.y = 0.0;
// 	obstacle_point.x = 100000;
// 	obstacle_point.y = -100000;
// 	float min_distance = 9999999;
// 	int idx = -1;

// 	if(obstacles->circles.size() == 0) {

}



// 	for(int i = 0 ; i < obstacles->circles.size() ; i++) {
// 		double length = cal_distance(car_point, obstacles->circles[i].center );
// 		if(min_distance > length) {

// 			min_distance = length;

// 			obstacle_point.x = obstacles->circles[i].center.x;
// 			obstacle_point.y = obstacles->circles[i].center.y;

// 		}

// 	}

// //	ROS_INFO_STREAM("x = " << obstacle_point.x << "y = " << obstacle_point.y);


// 	Point center_point;
// 	Point y_axis;
// 	Point circle;

// 	circle.x = obstacle_point.x;
// 	circle.y = obstacle_point.y;

// 	std::vector<Point> v1, v2;
// 	v1.push_back(center_point);
// 	v1.push_back(y_axis);
// 	v2.push_back(center_point);
// 	v2.push_back(circle);

// 	double angle = getAngle(circle.x, 0 , circle.x-1.0, circle.y);

// 	angle = angle * 180.0 / M_PI;

// 	steering = angle;

// 	if(steering >= 28) steering = 28;
// 	if(steering <= -28) steering -28;

// 	ROS_INFO_STREAM("steering = " << steering);
// }

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_node");

	ros::NodeHandle nh;

	ros::Subscriber obstacles_sub = nh.subscribe("obstacles", 10, obstacle_callback);

	ros::spin();
}