#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Int16.h"

#define SAFE_DISTANCE 27 //cm
#define SAFE_TO_TURN 30 //cm
//Full Rotation: 800 turns
//Wheel Circumference: 9.84 in
#define TURN_TURNS 500
#define BACKUP_TURNS 500

#define LINEAR_SPEED 1.1
#define ANGULAR_SPEED 10
#define ANGULAR_SPEED_CORRECTION (-0.1)

enum sonarLayout {
	SONAR_FRONT_RIGHT = 0,
	SONAR_FRONT_LEFT = 1,
	SONAR_LEFT = 2
};

enum motorStates {
	MOTOR_FORWARD,
	MOTOR_BACKWARDS,
	MOTOR_TURNING_LEFT,
	MOTOR_TURNING_RIGHT,
	MOTOR_STOP
};


class sensorUpdate {
public:
	sensorUpdate() = default;

	sensorUpdate &operator=(unsigned long long in) {
		data = in;
		newData = true;
		return *this;
	}

	operator unsigned long long() {
		newData = false;
		return data;
	}


	unsigned long long data = 0;

	bool newData = false;
private:
};

class SonarSensorExample {
public:
	SonarSensorExample();

private:
	ros::NodeHandle nh;
	ros::Timer timer;
	ros::Timer timer2;
	ros::Subscriber sonar_subs[3];
	ros::Subscriber encoder_subs[2];
	sensorUpdate sonarData[3];
	sensorUpdate encoderData[2];
	motorStates motorState;
	unsigned long long oldEncoders[2];
	bool isValidData = false;
	ros::Publisher vel_pub;
	geometry_msgs::Twist vel_msg;

	void processMotorState();

	motorStates oldState = MOTOR_STOP;
	ros::Subscriber go_sub;
	bool go;
};

SonarSensorExample::SonarSensorExample() {
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	go_sub = nh.subscribe<std_msgs::Bool>("/doSonar", 10, [=](const std_msgs::Bool::ConstPtr &msg){
		go=msg->data;
	});
	for (int i = 0; i < 3; i++) {
		std::stringstream s;
		s << "/arduino/sonar_" << i + 1;
		sonar_subs[i] = nh.subscribe<std_msgs::Int16>(s.str(), 10, [=](const std_msgs::Int16::ConstPtr &msg) {
			sonarData[i] = msg->data;
		});
	}
	for (int i = 0; i < 2; i++) {
		std::stringstream s;
		s << "/arduino/encoder_" << (i == 0 ? "left" : "right") << "_value";
		encoder_subs[i] = nh.subscribe<std_msgs::UInt64>(s.str(), 10, [=](const std_msgs::UInt64::ConstPtr &msg) {
			encoderData[i] = msg->data;
		});
	}
	for (int i = 0; i < 2; i++)oldEncoders[i] = 1;
	timer2 = nh.createTimer(ros::Duration(5.5),
							(const ros::TimerCallback &) [=](const ros::TimerEvent &t) { isValidData = true; });
	timer = nh.createTimer(ros::Duration(.1), (const ros::TimerCallback &) [=](const ros::TimerEvent &t) {
		bool hasUpdated = true;
		for (auto &i : sonarData)
			if (!i.newData)hasUpdated = false;
		for (auto &i : encoderData)
			if (!i.newData)hasUpdated = false;
		if (hasUpdated) {
			if (isValidData) {
				if (motorState == MOTOR_STOP) motorState = MOTOR_FORWARD;
				if (motorState == MOTOR_FORWARD) {
					if ((sonarData[SONAR_FRONT_LEFT] < SAFE_DISTANCE && sonarData[SONAR_FRONT_LEFT] != 0) ||
						(sonarData[SONAR_FRONT_RIGHT] < SAFE_DISTANCE && sonarData[SONAR_FRONT_RIGHT] != 0)) {
						motorState = MOTOR_BACKWARDS;
					}
					for (int i = 0; i < 2; i++) {
						if (oldEncoders[i] == encoderData[i] && oldState == MOTOR_FORWARD)
							motorState = MOTOR_BACKWARDS;
						oldEncoders[i] = encoderData[i];
					}
				} else if (motorState == MOTOR_BACKWARDS) {
					if ((sonarData[SONAR_FRONT_LEFT] > SAFE_TO_TURN || sonarData[SONAR_FRONT_LEFT] == 0) &&
						(sonarData[SONAR_FRONT_RIGHT] > SAFE_TO_TURN || sonarData[SONAR_FRONT_RIGHT] == 0) &&
						encoderData[1] - oldEncoders[1] > TURN_TURNS) {
						if (sonarData[SONAR_LEFT] > SAFE_TO_TURN || sonarData[SONAR_LEFT] == 0)
							motorState = MOTOR_TURNING_LEFT;
						else motorState = MOTOR_TURNING_RIGHT;

						for (int i = 0; i < 2; i++)oldEncoders[i] = encoderData[i];
					}
				} else if (motorState == MOTOR_TURNING_LEFT) {
					if (encoderData[1] - oldEncoders[1] > BACKUP_TURNS) {
						motorState = MOTOR_FORWARD;
					}
				} else if (motorState == MOTOR_TURNING_RIGHT) {
					if (encoderData[0] - oldEncoders[0] > BACKUP_TURNS) {
						motorState = MOTOR_FORWARD;
					}
				}
			} else motorState = MOTOR_STOP;
			ROS_INFO("Update! Motor Status: %d, Old Status: %d", motorState, oldState);
			processMotorState();
		}
		std::stringstream s;
		for (int i = 0; i < 2; i++) {
			s << i + 1 << ": " << encoderData[i] << "cm\t";
		}
		ROS_INFO("%s", s.str().c_str());
		oldState = motorState;
	});
}

void SonarSensorExample::processMotorState() {
	if(!go)return;
	switch (motorState) {
		case MOTOR_FORWARD:
			vel_msg.linear.x = LINEAR_SPEED;
			vel_msg.angular.z = ANGULAR_SPEED_CORRECTION;
			vel_pub.publish(vel_msg);
			break;
		case MOTOR_BACKWARDS:
			vel_msg.linear.x = -LINEAR_SPEED;
			vel_msg.angular.z = ANGULAR_SPEED_CORRECTION;
			vel_pub.publish(vel_msg);
			break;
		case MOTOR_TURNING_LEFT:
			vel_msg.linear.x = LINEAR_SPEED / 2.0;
			vel_msg.angular.z = ANGULAR_SPEED;
			vel_pub.publish(vel_msg);
			break;
		case MOTOR_TURNING_RIGHT:
			vel_msg.linear.x = LINEAR_SPEED / 2.0;
			vel_msg.angular.z = -ANGULAR_SPEED;
			vel_pub.publish(vel_msg);
			break;
		case MOTOR_STOP:
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			break;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sonar");

	ROS_INFO("Example Output");

	SonarSensorExample sonar;

	ros::spin();
}