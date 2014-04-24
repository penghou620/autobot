#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <string>

ros::Subscriber head_sub;
ros::Subscriber gesture_sub;
ros::Publisher cmd_vel_pub;
ros::Subscriber estop_sub;
ros::Subscriber mode_sub;
int start_flag = 0;
int estop = 0;
int auto_mode = 0;

void mode_callback(const std_msgs::String::ConstPtr& msg){
	//printf("mode: %s\n",msg->data.c_str());
	if(!strcmp(msg->data.substr(5).c_str(),"Auto")){
		auto_mode = 1;
		printf("Auto\n");
	}else{
		auto_mode = 0;
		printf("Manual\n");
	}
}
void estop_callback(const std_msgs::String::ConstPtr& msg){
	if(!strcmp(msg->data.c_str(), "set")){
		estop = 1;
		printf("Estop 1");
	}else{
		estop = 0;
		printf("Estop 0");
	}
}
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

void head_callback(const geometry_msgs::Point::ConstPtr& msg){
	if(auto_mode == 1 && estop == 0){
		std_msgs::String result;
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
						//float Vmax = 127.0;
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
						result.data = "A10B10";
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


