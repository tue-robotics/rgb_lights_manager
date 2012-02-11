/*
 * lights_manager.cpp
 *
 *  Created on: Feb 11, 2012
 *      Author: sdries
 */

#include "ros/ros.h"

#include "std_msgs/ColorRGBA.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"

using namespace std;

// color publisher
ros::Publisher pub_rgb_;

bool user_color_set_ = false;
std_msgs::ColorRGBA user_color;
std::string execution_state_;
bool base_status_ = true;
int arm_left_status_ = -1;   // default -1: no info
int arm_right_status_ = -1;  // default -1: no info
double head_status_ = -1;
bool emergency_switch_;

void update() {
	std_msgs::ColorRGBA rgb_msg;

	// default
	rgb_msg.r = 0;
	rgb_msg.g = 0;
	rgb_msg.b = 1;

	if (!base_status_) {
		// base not ok
	}

	if (arm_left_status_ == 0) {

	} else if (arm_left_status_ == 1) {

	} else {

	}

	// etc

	pub_rgb_.publish(rgb_msg);
}

void userCallback(const std_msgs::ColorRGBA::ConstPtr& rgb_msg) {
	user_color_set_ = true;
	user_color = *rgb_msg;
	update();
}

void execCallback(const std_msgs::String::ConstPtr& status_msg) {
	execution_state_ = status_msg->data;
	update();
}

void baseStatusCallback(const std_msgs::Bool::ConstPtr& status_msg) {
	base_status_ = status_msg->data;
	update();
}

void armLeftStatusCallback(const std_msgs::UInt8::ConstPtr& status_msg) {
	arm_left_status_ = status_msg->data;
	update();
}

void armRightStatusCallback(const std_msgs::UInt8::ConstPtr& status_msg) {
	arm_right_status_ = status_msg->data;
	update();
}

void headStatusCallback(const std_msgs::Float64::ConstPtr& status_msg) {
	head_status_ = status_msg->data;
	update();
}

void eSwitchCallback(const std_msgs::Bool::ConstPtr& status_msg) {
	emergency_switch_ = status_msg->data;
	update();
}



int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "rgb_lights_manager");
	ros::NodeHandle n("~");

	// Subscribe to the user rgb topic
	ros::Subscriber sub_user = n.subscribe("/user_rgb_lights", 1000, &userCallback);

	// Subscribe to the execution state topic
	ros::Subscriber sub_exec = n.subscribe("/execution_state", 1000, &execCallback);

	// Subscribe to hardware status
	ros::Subscriber sub_base = n.subscribe("/base_status", 1000, &baseStatusCallback);
	ros::Subscriber sub_arm_left = n.subscribe("/arm_left_status", 1000, &armLeftStatusCallback);
	ros::Subscriber sub_arm_right = n.subscribe("/arm_right_status", 1000, &armRightStatusCallback);
	ros::Subscriber sub_head = n.subscribe("/head_status", 1000, &headStatusCallback);

	// Subscribe to emergence switch
	ros::Subscriber sub_eswitch = n.subscribe("/emergency_switch", 1000, &eSwitchCallback);

	/*
	InputPort<std_msgs::Bool> eButtonPort, baseStatusPort,spindleStatusPort;
	InputPort<std_msgs::UInt8> peraLeftStatusPort, peraRightStatusPort;
	InputPort<std_msgs::Float64> headStatusPort;
	InputPort<std_msgs::String> executionStatePort;

	// Declare msgs used to reaed the input ports
	std_msgs::Bool eButtonPressed, baseStatus, spindleStatus;
	std_msgs::UInt8 peraLeftStatus, peraRightStatus;
	std_msgs::Float64 headStatus;
	std_msgs::String executionState;

	// Read the emergency button status
	eButtonPort.read(eButtonPressed);
	 */


	// Publisher
	pub_rgb_ = n.advertise<std_msgs::ColorRGBA>("/rgb_lights_controller/reference", 100);

	ros::spin();
}
