/**
 * @file joy_controller.cpp
 * @author Peng Hou
 * @date April 10, 2014
 * @brief This file interpret data coming from joystick and send out control command to motor controller and other 
 * @details This is written for a specific joystick(Logitech Gamepad F310). The buttons and axes may vary. 
 * Button 7: Clear estop flag, whenever estop is set, the robot wouldn't respond to the user command and would't move. Button 7 will clear the estop flag.
 * Button 8: Software Emergency stop button, whenever some thing goes wrong, pressing this button will send out a estop message to ROS topic "estop"
 * Button 5: Change the mode of the robot. When in manual mode, only joystick can control the robot. When in Auto mode, the joystick still works and the robot will respond to user actions
 * Axe 1(left stick vertical axe): This axe control the robot moving forward or backward. Pushing the stick straight up is forward and down is backward
 * Axe 3(right stick horizontal axe): This axe control the robot turning. Pushing the stick Left is turning left and right is turning right.
 */
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Point.h"
#include <string>

ros::Subscriber joy_sub;///<ROS subscriber subscribing to topic 'joy' which publishes joystick data
ros::Publisher cmd_vel_pub;///< ROS publisher publishes command velocity commands to topic "cmd_vel" which transfer these commands to serial data and send out to motor controller
ros::Publisher estop_pub;///< ROS publisher publishes estop message to topic "estop"
ros::Publisher gui_pub;///< ROS publisher publishes messages to topic "gui"
ros::Publisher mode_pub;///< ROS publisher publishes control mode to topic "mode"
int manual_mode = 1;///< default control mode is "manual"

/**
 * @brief callback for joystick data
 */

void joy_callback(const sensor_msgs::Joy &joy){
	std_msgs::String result;
	int button_8 = (int)(joy.buttons[8]);
	if(button_8 == 1){
		printf("estop button set\n");
		std_msgs::String estop_set;
		estop_set.data = "set";
		estop_pub.publish(estop_set);
	}

	int button_7 = (int)(joy.buttons[7]);
	if(button_7 == 1){
		printf("estop button clear\n");
		std_msgs::String estop_clear;
		estop_clear.data = "clear";
		estop_pub.publish(estop_clear);
	}
	int button_5 = joy.buttons[5];
	float axe_1 = (joy.axes[1]);//forward or backward
	float axe_3 = (joy.axes[3]);//turn left or right
	int Vmax = 0x1f;//set the maximum speed
	char command_string[7];

	std_msgs::String mode;
	if(button_5 == 1 && manual_mode == 0){
		manual_mode = 1;	
		mode.data = "Mode:Manual";
		mode_pub.publish(mode);
		gui_pub.publish(mode);
		printf("Manual mode\n");
	}else if(button_5 == 1 && manual_mode == 1){
		manual_mode = 0;	
		mode.data = "Mode:Auto";
		mode_pub.publish(mode);
		gui_pub.publish(mode);
		printf("Auto mode\n");
	}
	if(manual_mode == 1){//only send out velocity command when in manual mode
		if(axe_1 > 0.0){
			int velocity = (int)(Vmax * axe_1);//the velocity is percent of the maximum velocity
			if(velocity < 16){
				sprintf(command_string,"A0%xB0%x",velocity,velocity);
			}else{
				printf("velocity:%x\n",velocity);
				sprintf(command_string,"A%xB%x",velocity,velocity);
			}
			printf("command_string %s\n",command_string);
			result.data = command_string;

		}
		else if(axe_1 < 0.0){
			int velocity = (int)(Vmax * fabs(axe_1));
			if(velocity < 16){
				sprintf(command_string,"a0%xb0%x",velocity,velocity);
			}else{
				printf("velocity:%x\n",velocity);
				sprintf(command_string,"a%xb%x",velocity,velocity);
			}
			printf("command_string %s\n",command_string);
			result.data = command_string;
		}
		else if(axe_3 > 0.0){
			result.data ="A05B10";
		}
		else if(axe_3 < 0.0){
			result.data = "A10B05";
		}else{
			result.data = "A00B00";
		}
		cmd_vel_pub.publish(result);
	}
}
/**
 * @brief main
 */
int main(int argc, char **argv){
	ros::init(argc, argv, "joy_controller");
	ros::NodeHandle nh;
	joy_sub = nh.subscribe("joy",1000,joy_callback);
	cmd_vel_pub = nh.advertise<std_msgs::String>("cmd_vel",1);
	estop_pub = nh.advertise<std_msgs::String>("estop",1);
	gui_pub = nh.advertise<std_msgs::String>("gui",1);
	mode_pub = nh.advertise<std_msgs::String>("mode",1);
	ros::spin();
	return 0;
}


