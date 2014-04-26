/**
 * @file motor_controller.cpp
 * @author Peng Hou
 * @date April 10, 2014
 * @brief This file interpret data coming from vision and send out control command to motor controller and other 
 * @details The difference between this file and joy_controller is that this file determines the velocity commands based on data from the kinect.
 * The vision data is the position of the head of the user. The position consists of x,y,z coordinates. The velocities for the two wheels are calculated by equations:
 * 
 *	int left_vel = -1 * k*theta + Vmax/2;
 *	int right_vel = Vmax/2 + k*theta;
 * K is the proportional gain
 * Vmax is the maximum speed of the motor this can be set smaller to lower the speed down
 */

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <string>

ros::Subscriber head_sub;///< ROS subscriber for topic "head"
ros::Subscriber gesture_sub;///< ROS subscriber for topic "gesture"
ros::Publisher cmd_vel_pub;///< ROS publisher publishes velocity command to topic "cmd_vel"
ros::Subscriber estop_sub;///< ROS subscriber for topic "estop"
ros::Subscriber mode_sub;///< ROS subscriber for topic "mode"
int start_flag = 0;///< start flag determines whether the robot should follow the user. It will be set when start gesture is detected
int estop = 0;///< estop stops the robot immediately. It will be set when estop message is received
int auto_mode = 0;///< auto_mode indicates which mode the robot is operating on

/**
 * @brief callback for mode_sub. Set variable auto_mode when in auto mode
 */
void mode_callback(const std_msgs::String::ConstPtr& msg){
	if(!strcmp(msg->data.substr(5).c_str(),"Auto")){
		auto_mode = 1;
		printf("Auto\n");
	}else{
		auto_mode = 0;
		printf("Manual\n");
	}
}
/**
 * @brief callback for estop_sub. Set variable estop when in auto mode
 */

void estop_callback(const std_msgs::String::ConstPtr& msg){
	if(!strcmp(msg->data.c_str(), "set")){
		estop = 1;
		printf("Estop 1");
	}else{
		estop = 0;
		printf("Estop 0");
	}
}
/**
 * @brief callback for gesture_sub. Set variable start_flag when start gesture detected and clear start_flag when stop gesture detected
 */

void gesture_callback(const std_msgs::String::ConstPtr& msg){
	std::string start = "start";
	std::string stop = "stop";
	if(msg->data.compare(start) == 0){
		start_flag = 1;
		printf("msg %s\n",msg->data.c_str());
		printf("start flag set\n");
	}else if(msg->data.compare(stop) == 0){
		start_flag = 0;
		printf("msg %s\n",msg->data.c_str());
		printf("start flag clear\n");
	}
}
/**
 * @brief callback for head_sub. Whenver head coordinates data received, if in auto mode & estop is not set & start gesture received, it's safe to let the robot follow the user
 */

void head_callback(const geometry_msgs::Point::ConstPtr& msg){
	//if in automode and estop is not set, it's good to 
	if(auto_mode == 1 && estop == 0){
		std_msgs::String result;
		//if send out velocity command to control the robot
		if(start_flag == 1){
			float theta = atan2(-1*(msg->x),msg->z);//our model has the opposite coordinates frame with the kinect frame, so negative the x value
			printf("theta:%f\n",theta);
			printf("head x %f\n",msg->x);
			printf("head y %f\n",msg->y);
			printf("head z %f\n",msg->z);
			if(fabs(theta) >= 0.5){
				printf("Out of the range of the kinect\n");
				result.data = "A00B00";
				printf("velocity A00 B00\n");
			} else {
				if(msg->z <= 1200){
					printf("in the 1.5m\n");
					result.data = "A00B00";
					printf("velocity A00 B00\n");
				}else{
					if(fabs(theta) > 0.05){
						printf("Turn\n");
						float k=10.0;
						float Vmax = 20.0;
						int left_vel = -1 * k*theta + Vmax/2;
						int right_vel = Vmax/2 + k*theta;
						char tmp[7];
						if(left_vel < 16 && right_vel > 16){
							sprintf(tmp,"A0%xB%x",left_vel, right_vel);
						}else if(right_vel < 16 && left_vel > 16){
							sprintf(tmp,"A%xB0%x",left_vel, right_vel);
						}else if(left_vel < 16 && right_vel < 16){
							sprintf(tmp,"A0%xB0%x",left_vel, right_vel);
						}else{
							sprintf(tmp,"A%xB%x",left_vel, right_vel);
						}
						printf("Velocity Command %s\n",tmp);
						std::string tmp2(tmp);
						result.data = tmp2;
					} else {
						printf("Move Straight\n");
						result.data = "A10B10";//set constant moving speed
						printf("velocity A10 B10\n");
					}
				}
			}

		}else{
			printf("Stop Gesture\n");
			result.data = "A00B00";
			printf("velocity A00 B00\n");
		}
		cmd_vel_pub.publish(result);
	}
}
/**
 * @brief main
 */
int main(int argc, char **argv){
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle nh;
	head_sub = nh.subscribe("head",1,head_callback);
	gesture_sub = nh.subscribe("gesture",1,gesture_callback);
	estop_sub = nh.subscribe("estop",1,estop_callback);
	mode_sub = nh.subscribe("mode",1,mode_callback);
	cmd_vel_pub = nh.advertise<std_msgs::String>("cmd_vel",1);
	ros::Rate loop_rate(60);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


