/*
 * To control the motor, send velocity command to topic "cmd_vel" in the format of std_msgs::String
 * Example: A10B20 
 *			Channel A at velocity 10 and channel B at velocity 10
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <string>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int serial_fd = 0;
int n = 0;

int SerialOpen(){
   serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
   if (serial_fd == -1)
   {
     fprintf(stderr, "open_port: Unable to open /dev/ttyUSB0 - %s\n",
             strerror(errno));
     return -1;
   }
}

void cmd_vel_callback(const std_msgs::String::ConstPtr& msg){
	std::string left = msg->data.substr(0,3);
	std::string right = msg->data.substr(3,3);
	char tmp[6];
	sprintf(tmp,"!%s\r",left.c_str());
	n = write(serial_fd, tmp, 6);
	sprintf(tmp,"!%s\r",right.c_str());
	n = write(serial_fd, tmp, 6);
	if (n < 0){
  		fputs("write() of 4 bytes failed!\n", stderr);
  	}
}

int main(int argc, char **argv){
  if(SerialOpen() == -1){
  	return -1;
  }
  ros::init(argc, argv, "serial");

  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel",1000,cmd_vel_callback);

  ros::spin();
  return 0;
}
