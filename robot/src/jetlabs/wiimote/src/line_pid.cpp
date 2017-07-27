#include <math.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

geometry_msgs::Twist vel_msg;
std_msgs::Float32 error_msg;

ros::Publisher vel_pub;
ros::Subscriber line_error_sub;
ros::Subscriber go_sub;

float integral_error, derivative_error, cur_error, prev_error = 0;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
bool first_reconfig = true;
bool go = false;

void errorCallback(const std_msgs::Float32 &msg) {
	// Stop if the tape is no longer detected
	if(!go)return;
	if (msg.data > 12344 && msg.data < 12346) {
		vel_msg.angular.z = 0;
		vel_msg.linear.x = 0;
	} else {
		cur_error = msg.data;

		vel_msg.linear.x = 0.7;

		integral_error += cur_error;
		derivative_error = cur_error - prev_error;
		vel_msg.angular.z = -(Kp * cur_error + Kd * derivative_error + Ki * integral_error);
		ROS_INFO("Angular: %d", vel_msg.angular.z);
		prev_error = cur_error;
	}

	vel_pub.publish(vel_msg);
}
void goHandler(const std_msgs::Bool &msg){
	go = msg.data;
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "line_pid");
	ros::NodeHandle nh;

	line_error_sub = nh.subscribe("/line_error", 10, errorCallback);
	go_sub = nh.subscribe("/doFollow", 10, goHandler);

	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::spin();
}
