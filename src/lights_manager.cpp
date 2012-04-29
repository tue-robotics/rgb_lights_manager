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
#include "diagnostic_msgs/DiagnosticArray.h"
#include "smach_msgs/SmachContainerStatus.h"

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
Status diagnostic_status_ = NO_INFO;

// Function overloading allows taking the maximum of five values
Status max(Status a, Status b, Status c, Status d, Status e, Status f) {return max(max(max(a,b), max(c, max(d,e))),f);}

// String containing executive state
string state = "";

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

        // Set color using the state-to-color mapping
        if (execution_state_ != "" && colorMapping.count(execution_state_)>0) {
            state = execution_state_;
        } else state = "default";

        rgb_msg.r = colorMapping[state].r;
        rgb_msg.g = colorMapping[state].g;
        rgb_msg.b = colorMapping[state].b;

    }

    // Take the worst case hardware status
    int hardware_status = max(base_status_, arm_left_status_, arm_right_status_, head_status_,spindle_status_, diagnostic_status_);

    // If hardware status != normal, calculate sine for blinking/fading, otherwise, amp = 1
    double amp = hardware_status?0.5+0.5*sin(2 * 3.1415 * freq * (ros::Time::now().toSec()-t_start)):1;

    // If error, fading should be blinking, hence round sine to 0 or 1,
    if (hardware_status == 2) amp = floor(amp+0.5);

    rgb_msg.r *= amp;
    rgb_msg.g *= amp;
    rgb_msg.b *= amp;

    // Publish color
    pub_rgb_.publish(rgb_msg);
}

void userCallback(const amigo_msgs::RGBLightCommand::ConstPtr& rgb_msg) {
    // If the user gives a true, use the color that is published, if false, stop using the color
    user_color_set_ = rgb_msg->show_color.data;
    if (user_color_set_) user_color = rgb_msg->color;
}

template <typename T,unsigned S>
unsigned ArraySize(const T (&v)[S])
{
    return S;
}

void execCallback(const smach_msgs::SmachContainerStatus::ConstPtr& status_msg) {

    // Currently only listening to the first entry of the array of active states
    execution_state_ = status_msg->active_states[0];

    // Convert input to lower case only
    transform(execution_state_.begin(), execution_state_.end(), execution_state_.begin(), ptr_fun<int, int>(tolower));

    ///ROS_INFO("Execution state = %s",execution_state_.c_str());

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

void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    if (!strcmp(msg->status[0].name.c_str(),"Batteries")){

        if (msg->status[0].level == 0) {diagnostic_status_ = OK; }
        else if (msg->status[0].level == 1) {diagnostic_status_ = WARNING; }
        else {diagnostic_status_ = ERROR; }
    }

}

void initMapping() {
    colorMapping["default"] = RGB(0, 0, 1);       //Blue
    colorMapping["manipulate"] = RGB(1, 0, 0);    //Red
    colorMapping["navigate"] = RGB(0, 0, 1);	  //Blue
    colorMapping["hri"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["recognize"] = RGB(0.5, 0, 0.5); //Green
    colorMapping["idle"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["ebutton"] = RGB(1, 1, 0);       //No color
    colorMapping["download"] = RGB(0.2, 0.8, 0.2);      //Lime

    /*colorMapping["initialize"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["wait_for_door"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["enter_room"] = RGB(0, 0, 1);	  //Blue
    colorMapping["question"] = RGB(0.5, 0, 0.5); //Green
    colorMapping["explore"] = RGB(0, 0, 1);	  //Blue
    colorMapping["look"] = RGB(0.2, 0.8, 0.2);      //Lime
    colorMapping["pre_grab"] = RGB(1, 0, 0);    //Red
    colorMapping["prepare_orientation"] = RGB(1, 0, 0);    //Red
    colorMapping["grab"] = RGB(1, 0, 0);    //Red
    colorMapping["return"] = RGB(0, 0, 1);	  //Blue
    colorMapping["finish"] = RGB(0, 1, 1);		  //Light blue

    colorMapping["introduction"] = RGB(0.5, 0, 0.5); //Green
    colorMapping["follow"] = RGB(0, 0, 1); //Blue
    colorMapping["target_lost"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["wait"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["stop"] = RGB(0.5, 0, 0.5); //Green
    colorMapping["identify"] = RGB(0.5, 0, 0.5); //Green*/

    colorMapping["init"] = RGB(0.5, 0, 0.5); //
    colorMapping["await_person"] = RGB(0, 1, 1); //
    colorMapping["say_closer"] = RGB(0, 1, 0); //
    colorMapping["wait_1"] = RGB(0, 1, 1); //
    colorMapping["recognize_person"] = RGB(0, 1, 0); //
    colorMapping["say_hi"] = RGB(0, 1, 0); //
    colorMapping["say_dont_recognize"] = RGB(0, 1, 0); //
    colorMapping["check_if_operator"] = RGB(0.5, 0, 0.5); //
    colorMapping["say_wait_for_operator"] = RGB(0, 1, 0); //
    colorMapping["say_follow"] = RGB(0, 1, 0); //
    colorMapping["wait_for_check_loc"] = RGB(0, 1, 1); //
    colorMapping["check_operator_loc"] = RGB(0.5, 0, 0.5); //
    colorMapping["goto_room"] = RGB(0, 0, 1); //
    colorMapping["look_for_drink"] = RGB(0, 1, 0); //
    colorMapping["GrabMachine"] = RGB(1, 0, 0); //
    colorMapping["pre_grab"] = RGB(1, 0, 0); //
    colorMapping["prepare_orientation"] = RGB(1, 0, 0); //
    colorMapping["grab"] = RGB(1, 0, 0); //
    colorMapping["carrying_pose"] = RGB(1, 0, 0); //
    colorMapping["goto_person"] = RGB(0, 0, 1); //
    colorMapping["present_drink"] = RGB(0, 1, 0); //
    colorMapping["handover"] = RGB(1, 0, 0); //
    colorMapping["say_enjoy"] = RGB(0, 1, 0); //
    colorMapping["say_no_drink"] = RGB(0, 1, 0); //
    colorMapping["set_room_fail"] = RGB(0.5, 0, 0.5); //
    colorMapping["goto_person_failed"] = RGB(0, 0, 1); //
    colorMapping["report_failed"] = RGB(0, 1, 0); //
    colorMapping["return"] = RGB(0, 0, 1); //
    colorMapping["finish"] = RGB(0, 1, 1); //


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
    ros::Subscriber sub_exec = n.subscribe("/server_name/smach/container_status", 1, &execCallback);

    // Subscribe to hardware status
    ros::Subscriber sub_base = n.subscribe("/base_status", 1000, &baseStatusCallback);
    ros::Subscriber sub_arm_left = n.subscribe("/arm_left_status", 1000, &armLeftStatusCallback);
    ros::Subscriber sub_arm_right = n.subscribe("/arm_right_status", 1000, &armRightStatusCallback);
    ros::Subscriber sub_head = n.subscribe("/head_status", 1000, &headStatusCallback);
    ros::Subscriber sub_spindle = n.subscribe("/spindle_status", 1000, &spindleStatusCallback);

    // Subscribe to emergence switch
    ros::Subscriber sub_eswitch = n.subscribe("/emergency_switch", 1000, &eButtonCallback);

    // Subscribe to diagnostics topic
    ros::Subscriber diag_sub = n.subscribe("/diagnostics", 1000, &diagnosticCallback);

    // Publisher rgb value in interval [0,1]
    pub_rgb_ = n.advertise<std_msgs::ColorRGBA>("/rgb_lights_controller/reference", 100);

    ros::Rate r(10);
    while (ros::ok()) {

        // Set everything to default so only 'new' info is taken into account'
        base_status_ = NO_INFO;
        arm_left_status_ = NO_INFO;
        arm_right_status_ = NO_INFO;
        head_status_ = NO_INFO;
        spindle_status_ = NO_INFO;
        diagnostic_status_ = NO_INFO;

        state = "default";

        ros::spinOnce();
        update();
        r.sleep();
    }
}
