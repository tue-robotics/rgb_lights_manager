/*
 * lights_manager.cpp
 *
 *  Created on: Feb 11, 2012
 *      Author: sdries
 */

#include "ros/ros.h"

#include "std_msgs/ColorRGBA.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "tue_msgs/RGBLightCommand.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "smach_msgs/SmachContainerStatus.h"

#include <map>

using namespace std;

// Set frequency of blinking
const double frequency = 1.0;
double fade_freq = frequency;
double freq = frequency/2;

// Enumeration making the status easier to interpret
enum Status {
    NO_INFO = 0, // TODO can be given a unique number
    OK = 0,
    WARNING = 1,
    ERROR = 2,
    IDLE = 3
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
string state = "";
ros::Time time_execution_state_update_; // Time at which the execution has been last updated
ros::Duration execution_state_reset_duration_(60); // Time it takes before the execution state is reset to default

// Starting (ros) time of the node
double t_start = 0;

// Emergency button pressed by default, but should be updated continuously
bool emergency_button_pressed_ = true;

// Status hardware components, default is zero, meaning everything okay or switched off
Status hardware_status = NO_INFO;
Status diagnostic_status_ = NO_INFO;

// Function overloading allows taking the maximum of five values
//Status max(Status a, Status b, Status c, Status d, Status e, Status f) {return max(max(max(a,b), max(c, max(d,e))),f);}

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
    }
    else if (hardware_status == 3) { // if idle state show color equal to emergency button
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
        if (execution_state_ != "" && execution_state_ != "none" && colorMapping.count(execution_state_)>0) {
            state = execution_state_;
        } 
        // If execution_state_ == "none", keep the former color mapping
        else if (execution_state_ == "none") {
            ROS_DEBUG("Execution state = none");
        }
        else state = "default";

        if (colorMapping.find(state) != colorMapping.end()) {

            rgb_msg.r = colorMapping[state].r;
            rgb_msg.g = colorMapping[state].g;
            rgb_msg.b = colorMapping[state].b;
        } else {
            ROS_ERROR("MAPPING NOT FOUND: %s", state.c_str());
        }

    }

    // If hardware status != normal, calculate sine for blinking/fading, otherwise, amp = 1
    /*double amp = hardware_status?0.5+0.5*sin(2 * 3.1415 * freq * (ros::Time::now().toSec()-t_start)):1;

    // If error, fading should be blinking, hence round sine to 0 or 1,
    if (hardware_status == 2) amp = floor(amp+0.5);
     */
    double amp;
    if (hardware_status == 2){
        amp = 0.5+0.5*sin(2 * 3.1415 * freq * (ros::Time::now().toSec()-t_start));
        amp = floor(amp+0.5);
    }
    else if (hardware_status == 1){
        amp = 0.5+0.5*sin(2 * 3.1415 * fade_freq * (ros::Time::now().toSec()-t_start));
    }
    else {
        amp = 1;
    }

    rgb_msg.r *= amp;
    rgb_msg.g *= amp;
    rgb_msg.b *= amp;

    // Publish color
    pub_rgb_.publish(rgb_msg);
}

void userCallback(const tue_msgs::RGBLightCommand::ConstPtr& rgb_msg) {
    // If the user gives a true, use the color that is published, if false, stop using the color
    user_color_set_ = rgb_msg->show_color.data;
    if (user_color_set_) user_color = rgb_msg->color;
}

void execCallback(const smach_msgs::SmachContainerStatus::ConstPtr& status_msg) {

    if (!status_msg->active_states.empty()) {

        // Update time stamp at which execution state was last updated
        time_execution_state_update_ = ros::Time::now();

        // Currently only listening to the first entry of the array of active states
        execution_state_ = status_msg->active_states[0];
        ROS_DEBUG("%s",execution_state_.c_str());

        // Convert input to lower case only
        transform(execution_state_.begin(), execution_state_.end(), execution_state_.begin(), ptr_fun<int, int>(tolower));
    }
    else {
        ROS_WARN("Received empty status list");
    }
    ///ROS_INFO("Execution state = %s",execution_state_.c_str());
}

void hardwareCallback(const diagnostic_msgs::DiagnosticArray status_array) {
	int hardware_status_;
    hardware_status_ = max(status_array.status[1].level, max(status_array.status[1].level, max(status_array.status[2].level, max(status_array.status[3].level, max(status_array.status[4].level,status_array.status[5].level)))));
    //ROS_WARN("Took max status: %i. from array :[%i, %i, %i, %i, %i]!",hardware_status_,status_array.status[1].level,status_array.status[2].level,status_array.status[3].level,status_array.status[4].level,status_array.status[5].level);
    if (hardware_status_ == 0) { hardware_status = NO_INFO;	} 
    else if (hardware_status_ == 1) { hardware_status = IDLE; } 
    else if (hardware_status_ == 2) { hardware_status = OK; } 
    else if (hardware_status_ == 3) { hardware_status = WARNING; } 
    else { hardware_status = ERROR; }
    
}

void eButtonCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& status_msg) {
    emergency_button_pressed_ = false;
    for ( int i=0; i<status_msg->status.size(); i++ ){
        if ( status_msg->status[i].level )
            emergency_button_pressed_ = true;
    }
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
    colorMapping["recognize"] = RGB(0, 1, 0); //Green
    colorMapping["idle"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["ebutton"] = RGB(1, 1, 0);       //No color
    colorMapping["download"] = RGB(0.2, 0.8, 0.2);      //Lime

    /// Clean up
    colorMapping["initialize"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["wait_for_door"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["enter_room"] = RGB(0, 0, 1);	  //Blue
    colorMapping["question"] = RGB(0, 1, 0); //Green
    colorMapping["explore"] = RGB(0, 0, 1);	  //Blue
    colorMapping["look"] = RGB(0.2, 0.8, 0.2);      //Lime
    colorMapping["pre_grab"] = RGB(1, 0, 0);    //Red
    colorMapping["prepare_orientation"] = RGB(1, 0, 0);    //Red
    colorMapping["grab"] = RGB(1, 0, 0);    //Red
    colorMapping["return"] = RGB(0, 0, 1);	  //Blue
    colorMapping["finish"] = RGB(0, 1, 1);		  //Light blue

    /// Follow Me
    colorMapping["initialize"] = RGB(0.5, 0, 0.5); //Purple
    colorMapping["introduction"] = RGB(0, 1, 0); //Green
    colorMapping["learn_face"] = RGB(0, 1, 0); //Green
    colorMapping["follow"] = RGB(0, 0, 1); //Blue
    colorMapping["target_lost"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["wait"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["stop"] = RGB(0, 1, 0); //Green
    colorMapping["identify"] = RGB(0, 1, 0); //Green

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

    /// Who is Who
    colorMapping["init"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["instruct_wait_for_door"] = RGB(0, 1, 0); //Green
    colorMapping["await_door_open"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["store_time"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["set_first_target"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["enter_first_room"] = RGB(0, 0, 1);       //Blue
    colorMapping["learn_single_person"] = RGB(0, 1, 0); //Green
    colorMapping["store_name"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["intro_of_guests"] = RGB(0, 1, 0); //Green
    colorMapping["instruct_wait_for_continue"] = RGB(0, 1, 0); //Green
    colorMapping["await_continue"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["set_people_room_as_target"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["goto_second_room"] = RGB(0, 0, 1);       //Blue
    colorMapping["instruct_ordering"] = RGB(0, 1, 0); //Green
    colorMapping["await_orderer"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["say_closer"] = RGB(0, 1, 0); //Green
    colorMapping["recognize_person"] = RGB(0, 1, 0); //Green
    colorMapping["say_hi"] = RGB(0, 1, 0); //Green
    colorMapping["generate_ask_order_sentence"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["ask_drink"] = RGB(0, 1, 0); //Green
    colorMapping["store_order"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["take_single_order"] = RGB(0, 1, 0); //Green
    colorMapping["take_orders"] = RGB(0, 1, 0); //Green
    colorMapping["recite_orders"] = RGB(0, 1, 0); //Green
    colorMapping["say_deliver_drink"] = RGB(0, 1, 0); //Green
    colorMapping["set_drinks_room_as_targets"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["goto_drinks_room"] = RGB(0, 0, 1);       //Blue
    colorMapping["set_desired_objects"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["explore_drinks"] = RGB(0, 0, 1);       //Blue
    colorMapping["look_for_drinks"] = RGB(0.2, 0.8, 0.2);      //Lime
    colorMapping["store_failed_drink"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["inform_failed_drink"] = RGB(0, 1, 0); //Green
    colorMapping["pre_grab"] = RGB(1, 0, 0);    //Red
    colorMapping["human_handover"] = RGB(0, 1, 0); //Green
    colorMapping["prepare_orientation"] = RGB(1, 0, 0);    //Red
    colorMapping["grab"] = RGB(1, 0, 0);    //Red
    colorMapping["carrying_pose"] = RGB(1, 0, 0);    //Red
    colorMapping["set_people_room_as_target_2"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["goto_persons"] = RGB(0, 0, 1);       //Blue
    colorMapping["set_drinks_receiver"] = RGB(0.5, 0, 0.5);		  //Purple
    colorMapping["identify_receiver"] = RGB(0, 1, 0); //Green
    colorMapping["say_drink_delivered"] = RGB(0, 1, 0); //Green
    colorMapping["wait_for_thanks"] = RGB(0, 1, 1);		  //Light blue
    colorMapping["release_drink"] = RGB(1, 0, 0);    //Red
    colorMapping["say_take_drink"] = RGB(0, 1, 0); //Green
    colorMapping["report_failed_drinks"] = RGB(0, 1, 0); //Green
    colorMapping["reset_arm"] = RGB(1, 0, 0);    //Red
    colorMapping["deliver_single_drink"] = RGB(0, 1, 0); //Green
    colorMapping["deliver_drinks"] = RGB(0, 1, 0); //Green
    colorMapping["goto_persons_2"] = RGB(0, 0, 1);       //Blue
    colorMapping["report_failed_drinks"] = RGB(0, 1, 0); //Green
    colorMapping["return"] = RGB(0, 0, 1);       //Blue
    colorMapping["finish"] = RGB(0, 1, 1);		  //Light blue

    /// Open challenge 2012
    colorMapping["init"] = RGB(0.5, 0, 0.5);                  //Purple
    colorMapping["say_intro1"] = RGB(0, 1, 0); //Green
    colorMapping["say_intro2"] = RGB(0, 1, 0); //Green
    colorMapping["say_intro3"] = RGB(0, 1, 0); //Green
    colorMapping["goto_startpoint"] = RGB(0, 0, 1);       //Blue
    colorMapping["await_person"] = RGB(0, 1, 1);           //Light blue
    colorMapping["ask_to_get_drink"] = RGB(0, 1, 0); //Green
    colorMapping["say_get_drink_anyway"] = RGB(0, 1, 0); //Green
    colorMapping["goto_table"] = RGB(0, 0, 1);       //Blue
    colorMapping["look_for_objects"] = RGB(0.2, 0.8, 0.2);      //Lime
    colorMapping["say_no_drink"] = RGB(0, 1, 0); //Green
    colorMapping["say_help"] = RGB(0, 1, 0); //Green
    colorMapping["check_coke_presence"] = RGB(0.5, 0, 0.5);            //Purple
    colorMapping["grab_left"] = RGB(1, 0, 0);    //Red
    colorMapping["check_cup_presence"] = RGB(0.5, 0, 0.5);            //Purple
    colorMapping["grab_right"] = RGB(1, 0, 0);    //Red
    colorMapping["check_objects"] = RGB(0.5, 0, 0.5);            //Purple
    colorMapping["say_missed_left"] = RGB(0, 1, 0); //Green
    colorMapping["say_missed_right"] = RGB(0, 1, 0); //Green
    colorMapping["say_missed_both"] = RGB(0, 1, 0); //Green
    colorMapping["wait_for_one_object"] = RGB(0, 1, 1);           //Light blue
    colorMapping["wait_for_two_objects"] = RGB(0, 1, 1);           //Light blue
    colorMapping["prepare_poor"] = RGB(1, 0, 0);    //Red
    colorMapping["say_cant_poor"] = RGB(0, 1, 0); //Green
    colorMapping["poor"] = RGB(1, 0, 0);    //Red
    colorMapping["say_trashbin"] = RGB(0, 1, 0); //Green
    colorMapping["goto_trashbin"] = RGB(0, 0, 1);       //Blue
    colorMapping["drop_can"] = RGB(1, 0, 0);    //Red
    colorMapping["goto_dropoff"] = RGB(0, 0, 1);       //Blue
    colorMapping["say_dropoff"] = RGB(0, 1, 0); //Green
    colorMapping["dropoff"] = RGB(1, 0, 0);    //Red
    colorMapping["reset_arm_after_dropoff"] = RGB(1, 0, 0);    //Red
    colorMapping["say_dropoff_failed"] = RGB(0, 1, 0); //Green
    colorMapping["ask_other_tasks"] = RGB(0, 1, 0); //Green
    colorMapping["store_task_answer"] = RGB(0.5, 0, 0.5);            //Purple
    colorMapping["say_been_too_busy"] = RGB(0, 1, 0); //Green
    colorMapping["say_no_answer_other_task"] = RGB(0, 1, 0); //Green
    colorMapping["say_been_too_busy"] = RGB(0, 1, 0); //Green
    colorMapping["say_move_to_exit"] = RGB(0, 1, 0); //Green
    colorMapping["goto_exit"] = RGB(0, 0, 1);       //Blue
    colorMapping["finish"] = RGB(0, 1, 1);           //Light blue

}


int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "rgb_lights_manager");
    ros::NodeHandle n("~");
    ros::NodeHandle gn;

    // Initialize state-to-color mapping
    initMapping();

    // Store starting time
    t_start = ros::Time::now().toSec();
    time_execution_state_update_ = ros::Time::now();

    // Subscribe to the user rgb topic
    ros::Subscriber sub_user = n.subscribe("user_set_rgb_lights", 1, &userCallback);

    // Subscribe to the execution state topic
    ros::Subscriber sub_exec = gn.subscribe("smach/container_status", 1, &execCallback);

    // Subscribe to hardware status
    ros::Subscriber sub_hardware = gn.subscribe("hardware_status", 1, &hardwareCallback);

    // Subscribe to emergence switch
    ros::Subscriber sub_eswitch = gn.subscribe("ebutton_status", 1, &eButtonCallback);

    // Subscribe to diagnostics topic
    ros::Subscriber diag_sub = gn.subscribe("/diagnostics", 1, &diagnosticCallback);

    // Publisher rgb value in interval [0,1]
    pub_rgb_ = gn.advertise<std_msgs::ColorRGBA>("rgb_lights_controller/reference", 100);

    ros::Rate r(20);
    while (ros::ok()) {

        // Set everything to default so only 'new' info is taken into account'
        hardware_status = NO_INFO;
		diagnostic_status_ = NO_INFO;

        // Only set execution state to default if no new info has been received for a certain duration
        // This prevents the lights from turning blue all the time
        if (((ros::Time::now() - time_execution_state_update_) > execution_state_reset_duration_) && execution_state_ != "default" ) {
            ROS_INFO("Resetting execution state to default");
            execution_state_ = "default";
        }

        ros::spinOnce();
        update();
        r.sleep();
    }
}
