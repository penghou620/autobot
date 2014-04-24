#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Point.h"
#include <string>

ros::Subscriber joy_sub;
ros::Publisher cmd_vel_pub;
ros::Publisher estop_pub;
ros::Publisher gui_pub;
ros::Publisher mode_pub;
int prev_button_7 = 0;
int manual_mode = 0;

void joy_callback(const sensor_msgs::Joy &joy){
	std_msgs::String result;
	int button_7 = (int)(joy.buttons[7]);
	if(button_7 == 1){
		std_msgs::String estop_clear;
		estop_clear.data = "clear";
		estop_pub.publish(estop_clear);
	}
	int button_1 = joy.buttons[5];
	float axe_1 = (joy.axes[1]);//forward or backward
	float axe_3 = (joy.axes[3]);//turn left or right
	int Vmax = 0x1f;
	char command_string[7];

	std_msgs::String mode;
	if(button_1 == 1 && manual_mode == 0){
		manual_mode = 1;	
		mode.data = "Mode:Manual";
		gui_pub.publish(mode);
		printf("manual mode\n");
	}else if(button_1 == 1 && manual_mode == 1){
		manual_mode = 0;	
		printf("auto mode\n");
		mode.data = "Mode:Auto";
		gui_pub.publish(mode);
	}
	if(manual_mode == 1){
		if(axe_1 > 0.0){
			int velocity = (int)(Vmax * axe_1);
			if(velocity < 16){
				//velocity = 0;
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
				//velocity = 0;
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
//	else{
//		if(prev_button_7 == 1){
//			prev_button_7 = 0;
//			result.data = "A00B00";
//			cmd_vel_pub.publish(result);
//		}
//	}
}
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


