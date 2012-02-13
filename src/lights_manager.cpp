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
#include "amigo_msgs/RGBLightCommand.h"

#include <map>

using namespace std;

// Set frequency of blinking
const double frequency = 1.0;
double freq = frequency/2;

// Enumeration making the status easier to interpret
enum Status {
	NO_INFO = 0, // TODO can be given a unique number
	OK = 0,
	WARNING = 1,
	ERROR = 2
};

// Class that contains an RGB color
class RGB {
public:
	double r, g, b;

	RGB() : r(0), g(0), b(0) {}
	RGB(double r1, double g1, double b1) : r(r1), g(g1), b(b1) {}

};

// Color publisher
ros::Publisher pub_rgb_;

// Mapping from state to color
map<string, RGB > colorMapping;

// Boolean indicating if a color from a user should be used
bool user_color_set_ = false;

// Color used to store color given by user
std_msgs::ColorRGBA user_color;

// State received from executioner, empty by default
string execution_state_ = "";

// Starting (ros) time of the node
double t_start = 0;

// Emergency button pressed by default, but should be updated continuously
bool emergency_button_pressed_ = true;

// Status hardware components, default is zero, meaning everything okay or switched off
Status base_status_ = NO_INFO;
Status arm_left_status_ = NO_INFO;
Status arm_right_status_ = NO_INFO;
Status head_status_ = NO_INFO;
Status spindle_status_ = NO_INFO;

// Function overloading allows taking the maximum of five values
Status max(Status a, Status b, Status c, Status d, Status e) {return max(max(a,b), max(c, max(d,e)));}


/* Function that generates an output on the /rgb_lights_controller/reference topic based on all inputs
 * Priority list:
 * 1. emergency button
 * 2. user input
 * 3. state from executioner, in this case
 *    - normal: static color (or all hardware turned off)
 *    - warning: fading color
 *    - error: blinking color
 */
void update() {

	// Output message, alpha is et to one by default
	std_msgs::ColorRGBA rgb_msg;
	rgb_msg.a = 1;

	// If emergency button pressed, take the ebutton color
	if (emergency_button_pressed_) {
		rgb_msg.r = colorMapping["ebutton"].r;
		rgb_msg.g = colorMapping["ebutton"].g;
		rgb_msg.b = colorMapping["ebutton"].b;
	} // If a user defined a color, use that color
	else if (user_color_set_) {
		rgb_msg.r = user_color.r;
		rgb_msg.g = user_color.g;
		rgb_msg.b = user_color.b;
		// If user gives a non-zero alpha, then use it
		if (!user_color.a) rgb_msg.a = user_color.a;
	} // If no user input is given, base color on executioner state
	else {

		// Take the worst case hardware status
		int hardware_status = max(base_status_, arm_left_status_, arm_right_status_, head_status_,spindle_status_);

		// Set color using the state-to-color mapping
		string state = "";
		if (execution_state_ != "" && colorMapping.count(execution_state_)>0) {
			state = execution_state_;
		} else state = "default";

		rgb_msg.r = colorMapping[state].r;
		rgb_msg.g = colorMapping[state].g;
		rgb_msg.b = colorMapping[state].b;

		// If hardware status != normal, calculate sine for blinking/fading, otherwise, amp = 1
		double amp = hardware_status?0.5+0.5*sin(2 * 3.1415 * freq * (ros::Time::now().toSec()-t_start)):1;

		// If error, fading should be blinking, hence round sine to 0 or 1,
		if (hardware_status == 2) amp = floor(amp+0.5);

		rgb_msg.r *= amp;
		rgb_msg.g *= amp;
		rgb_msg.b *= amp;

	}

	// Publish color
	pub_rgb_.publish(rgb_msg);
}

void userCallback(const amigo_msgs::RGBLightCommand::ConstPtr& rgb_msg) {
	// If the user gives a true, use the color that is published, if false, stop using the color
	user_color_set_ = rgb_msg->show_color.data;
	if (user_color_set_) user_color = rgb_msg->color;
}

void execCallback(const std_msgs::String::ConstPtr& status_msg) {
	execution_state_ = status_msg->data;
	// Convert input to lower case only
	transform(execution_state_.begin(), execution_state_.end(), execution_state_.begin(), ptr_fun<int, int>(tolower));
}

void baseStatusCallback(const std_msgs::Bool::ConstPtr& status_msg) {
	// status_msg->data is true means base is ok
	base_status_ = (status_msg->data)?OK:ERROR;
}

void armLeftStatusCallback(const std_msgs::UInt8::ConstPtr& status_msg) {
	if (status_msg->data == 0) arm_left_status_ = OK;
	else if (status_msg->data == 1) arm_left_status_ = WARNING;
	else arm_left_status_ = ERROR;
}

void armRightStatusCallback(const std_msgs::UInt8::ConstPtr& status_msg) {
	if (status_msg->data == 0) arm_right_status_ = OK;
	else if (status_msg->data == 1) arm_right_status_ = WARNING;
	else arm_right_status_ = ERROR;
}

void headStatusCallback(const std_msgs::Float64::ConstPtr& status_msg) {
	// TODO this topic gives meaningless data, therefore always consider it to be correct
	head_status_ = OK;
}

void spindleStatusCallback(const std_msgs::Bool::ConstPtr& status_msg) {
	// status_msg->data is true means spindle is ok
	spindle_status_ = (status_msg->data)?OK:ERROR;
}

void eButtonCallback(const std_msgs::Bool::ConstPtr& status_msg) {
	emergency_button_pressed_ = (status_msg->data);
}

void initMapping() {
	colorMapping["default"] = RGB(0, 0, 1);       //Blue
	colorMapping["manipulate"] = RGB(1, 0, 0);    //Red
	colorMapping["navigate"] = RGB(0, 0, 1);	  //Blue
	colorMapping["hri"] = RGB(0.5, 0, 0.5);		  //Purple
	colorMapping["recognize"] = RGB(0.5, 0, 0.5); //Green
	colorMapping["idle"] = RGB(0, 1, 1);		  //Light blue
	colorMapping["ebutton"] = RGB(1, 1, 0);       //No color
	colorMapping["download"] = RGB(1, 0, 1);      //Lime

	//colorMapping["brown"] = RGB(0.36, 0.2, 0.09);
	//colorMapping["white"] = RGB(1, 1, 1);
	//colorMapping["yellow"] = RGB(1, 1, 0);

}


int main(int argc, char **argv) {

	// Initialize node
	ros::init(argc, argv, "rgb_lights_manager");
	ros::NodeHandle n("~");

	// Initialize state-to-color mapping
	initMapping();

	// Store starting time
	t_start = ros::Time::now().toSec();

	// Subscribe to the user rgb topic
	ros::Subscriber sub_user = n.subscribe("/user_set_rgb_lights", 1000, &userCallback);

	// Subscribe to the execution state topic
	ros::Subscriber sub_exec = n.subscribe("/execution_state", 1000, &execCallback);

	// Subscribe to hardware status
	ros::Subscriber sub_base = n.subscribe("/base_status", 1000, &baseStatusCallback);
	ros::Subscriber sub_arm_left = n.subscribe("/arm_left_status", 1000, &armLeftStatusCallback);
	//ros::Subscriber sub_arm_right = n.subscribe("/arm_right_status", 1000, &armRightStatusCallback);
	ros::Subscriber sub_head = n.subscribe("/head_status", 1000, &headStatusCallback);
	ros::Subscriber sub_spindle = n.subscribe("/spindle_status", 1000, &spindleStatusCallback);

	// Subscribe to emergence switch
	ros::Subscriber sub_eswitch = n.subscribe("/emergency_switch", 1000, &eButtonCallback);

	// Publisher rgb value in interval [0,1]
	pub_rgb_ = n.advertise<std_msgs::ColorRGBA>("/rgb_lights_controller/reference", 100);

	ros::Rate r(20);
	while (ros::ok()) {
		ros::spinOnce();
		update();
		r.sleep();
	}
}
