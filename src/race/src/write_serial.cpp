#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>
#include <string>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <sstream>


serial::Serial ser;

using namespace std;

unsigned char alive=0x00;
unsigned char gear = 0x00;
unsigned char speed_0 = 0x00;
unsigned char speed_1 = 0x00;
unsigned char steer_0 = 0x00;
unsigned char steer_1 = 0x00;
unsigned char front_brake = 0x01;
unsigned char estop = 0x00;

void serialCallback(const race::drive_values::ConstPtr& msg) {
	int steer_total = 0;
	unsigned int speed_total = 0;
	speed_total = abs(msg->throttle)*10;
	printf("msg->throttle : %d\n", msg->throttle);
	if(msg->throttle < 255 && msg->throttle > 0) { // forward
		gear = 0x00;
		speed_1 = speed_total;
		speed_0 = 0x00;
		front_brake = 0x00;
		estop = 0x00;
	} else if(msg->throttle > -255 && msg->throttle < 0) { // backward
		gear = 0x02;
		speed_1 = speed_total;
		speed_0 = 0x00;
		front_brake = 0x00;
		estop = 0x00;
	} else if(msg->throttle == 0) { // stop
		speed_0 = 0x00;
		speed_1 = 0x00;
		gear = 0x01;
		front_brake = 0x33;
		estop = 0x01;
	} else{
		gear = 0x01;
		speed_0 = 0x00;
		speed_1 = 0x00;
		front_brake = 0x33;
		estop = 0x01;
	}
	steer_total = ((float)(msg->steering) * 71.0 * -1);
	printf("steer : %d , speed : %u\n", steer_total, speed_total);
	steer_0 = steer_total >> 8;
	steer_1 = steer_total & 0xff;
}


int main (int argc, char** argv) {
	unsigned char str[14] = {0x53, 0x54, 0x58, 0x01, estop, 0x00, speed_0, speed_1, steer_0, steer_1, front_brake, alive, 0x0D, 0x0A};
	ros::init(argc, argv, "serial_node");
	ros::NodeHandle nh;

	ros::Subscriber control_value_sub = nh.subscribe("control_value", 10, serialCallback);

//	try {
//		ser.setPort("/dev/ttyUSB0");
//		ser.setBaudrate(115200);
//		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//		ser.setTimeout(to);
//		ser.open();
//	} catch(serial::IOException& e) {
//		ROS_ERROR_STREAM("Unable to open port");
//		return -1;
//	}
//
//	if(ser.isOpen()) ROS_INFO_STREAM("Serial Port initialized");
//	else return -1;

	ros::Rate loop_rate(50);
	size_t num_write = 14;

	while(ros::ok()) {
		ros::spinOnce();
		str[4]  = estop;
		str[5]  = gear;
		str[6]  = speed_0;
		str[7]  = speed_1;
		str[8]  = steer_0;
		str[9]  = steer_1;
		str[10] = front_brake;
		str[11] = alive;
		
		//ser.write(str, num_write);

		if(alive != 0xff) alive++;
		else alive = 0x00;

		loop_rate.sleep();
	}
	ser.close();
}
