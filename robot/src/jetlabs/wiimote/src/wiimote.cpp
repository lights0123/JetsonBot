#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"
#include <csignal>
#include <poll.h>
#include <std_msgs/Bool.h>
#include "xwiimote.h"

#define SONAR_OBJECT_CLOSE 35
#define SONAR_OBJECT_CLOSE_BOOST 50
const double LINEAR_SPEED[5] = {0.5, 0.8, 1.2, 1.5, 10};
const double ANGULAR_SPEED[5] = {0.5, 0.8, 1.2, 1.5, 10};
static struct xwii_iface *iface;
int frontLeftSonar;
int frontRightSonar;
ros::Subscriber sonar_subs[2];
ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;
ros::Timer timerLED;
std_msgs::Bool go;
std_msgs::Bool goSonar;
ros::Publisher go_pub;
ros::Publisher sonar_pub;

void mySigintHandler(int sig) {
	xwii_iface_unref(iface);
	ros::shutdown();
}

char *get_dev(int num) {
	struct xwii_monitor *mon;
	char *ent;
	int i = 0;

	mon = xwii_monitor_new(false, false);
	if (!mon) {
		printf("Cannot create monitor\n");
		return NULL;
	}

	while ((ent = xwii_monitor_poll(mon))) {
		if (++i == num)
			break;
		free(ent);
	}

	xwii_monitor_unref(mon);

	if (!ent)
		printf("Cannot find device with number #%d\n", num);

	return ent;
}

int speed = 1;

void updateLEDs() {
	for (int i = 1; i <= speed; i++) xwii_iface_set_led(iface, XWII_LED(i), true);
	for (int i = 4; i > speed; i--) xwii_iface_set_led(iface, XWII_LED(i), false);
}

void changeSpeed(bool increase) {
	if (increase) {
		if (speed < 4) speed++;
	} else if (speed > 1) speed--;
	updateLEDs();
}

void handle_watch(void) {
	static unsigned int num;
	int ret;

	ROS_INFO("Info: Watch Event #%u", ++num);

	ret = xwii_iface_open(iface, xwii_iface_available(iface) |
								 XWII_IFACE_WRITABLE);
	if (ret)
		ROS_ERROR("Error: Cannot open interface: %d", ret);

}

bool isKeyDown = false;
unsigned int keyDownKey = NULL;
bool rumble = false;
bool isSpeedBoost = false;
enum controlStates {
	CONTROL_WII,
	CONTROL_FACE,
	CONTROL_SONAR
};
controlStates controlState = CONTROL_WII;

bool ledsOn = false;

void runLEDs(const ros::TimerEvent &) {
	ROS_INFO("LED");
	switch (controlState) {
		case CONTROL_WII:
			updateLEDs();
			break;
		case CONTROL_FACE:
		case CONTROL_SONAR:
			for (int i = 1; i <= 4; i++) xwii_iface_set_led(iface, XWII_LED(i), ledsOn);
			ledsOn = !ledsOn;
	}
}
void run_iface(const ros::TimerEvent &) {
	struct xwii_event event;
	int ret = 0, fds_num;
	struct pollfd fds[2];

	memset(fds, 0, sizeof(fds));
	fds[0].fd = 0;
	fds[0].events = POLLIN;
	fds[1].fd = xwii_iface_get_fd(iface);
	fds[1].events = POLLIN;
	fds_num = 2;

	ret = xwii_iface_watch(iface, true);
	if (ret)
		ROS_ERROR("Error: Cannot initialize hotplug watch descriptor");

	ret = poll(fds, fds_num, -1);
	if (ret < 0) {
		if (errno != EINTR) {
			ret = -errno;
			ROS_ERROR("Error: Cannot poll fds: %d", ret);
		}
	}

	ret = xwii_iface_dispatch(iface, &event, sizeof(event));
	if (ret) {
		if (ret != -EAGAIN) {
			ROS_ERROR("Error: Read failed with err:%d",
					  ret);
		}
	} else {
		switch (event.type) {
			case XWII_EVENT_GONE:
				ROS_ERROR("Info: Device gone");
				fds[1].fd = -1;
				fds[1].events = 0;
				fds_num = 1;
				break;
			case XWII_EVENT_WATCH:
				handle_watch();
				break;
			case XWII_EVENT_KEY:
				unsigned int code = event.v.key.code;
				bool pressed = event.v.key.state;
				if (code == XWII_KEY_LEFT || code == XWII_KEY_RIGHT || code == XWII_KEY_UP || code == XWII_KEY_DOWN) {
					if (controlState != CONTROL_WII)break;
					if (pressed && !isKeyDown) {
						isKeyDown = true;
						keyDownKey = code;
					} else if (!pressed && isKeyDown && keyDownKey == code) {
						isKeyDown = false;
					}
				} else if (code == XWII_KEY_MINUS) {
					if (controlState != CONTROL_WII)break;
					if (pressed)
						changeSpeed(false);
				} else if (code == XWII_KEY_PLUS) {
					if (controlState != CONTROL_WII)break;
					if (pressed)
						changeSpeed(true);
				} else if (code == XWII_KEY_B) {
					isSpeedBoost = pressed;
				} else if (code == XWII_KEY_A) {
					if (pressed) {
						switch(controlState){
							case CONTROL_WII:
								controlState = CONTROL_FACE;
								break;
							case CONTROL_FACE:
								controlState = CONTROL_SONAR;
								break;
							case CONTROL_SONAR:
								controlState = CONTROL_WII;
								break;
						}
						go.data = false;
						goSonar.data = false;
						switch (controlState) {
							case CONTROL_WII:
								updateLEDs();
								timerLED.stop();
								break;
							case CONTROL_FACE:
								timerLED.setPeriod(ros::Duration(0.6), false);
								ledsOn = false;
								runLEDs(ros::TimerEvent());
								timerLED.start();
								ROS_INFO("Face");
								go.data = true;
								break;
							case CONTROL_SONAR:
								timerLED.setPeriod(ros::Duration(0.25), false);
								ledsOn = false;
								runLEDs(ros::TimerEvent());
								timerLED.start();
								goSonar.data = true;
								break;
						}
						go_pub.publish(go);
						sonar_pub.publish(goSonar);
					}
				}
				break;
		}
		if (isKeyDown) {
			switch (keyDownKey) {
				case XWII_KEY_UP:
					vel_msg.linear.x = LINEAR_SPEED[isSpeedBoost ? 4 : speed - 1];
					vel_msg.angular.z = 0;
					break;
				case XWII_KEY_DOWN:
					vel_msg.linear.x = -LINEAR_SPEED[isSpeedBoost ? 4 : speed - 1];
					vel_msg.angular.z = 0;
					break;
				case XWII_KEY_LEFT:
					vel_msg.linear.x = 0.7;
					vel_msg.angular.z = ANGULAR_SPEED[isSpeedBoost ? 4 : speed - 1];
					break;
				case XWII_KEY_RIGHT:
					vel_msg.linear.x = 0.7;
					vel_msg.angular.z = -ANGULAR_SPEED[isSpeedBoost ? 4 : speed - 1];
					break;
			}
			std::string s;
			switch (keyDownKey) {
				case XWII_KEY_LEFT:
					s = "left";
					break;
				case XWII_KEY_RIGHT:
					s = "right";
					break;
				case XWII_KEY_UP:
					s = "up";
					break;
				case XWII_KEY_DOWN:
					s = "down";
					break;
			}

			//ROS_INFO("Key Down: %s", s.c_str());
		} else {
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			//ROS_INFO("No Key, speed %d, left %d, right %d", speed, frontLeftSonar, frontRightSonar);
		}
		if (controlState == CONTROL_WII) vel_pub.publish(vel_msg);
	}
	if ((isSpeedBoost && vel_msg.linear.x) ? (frontLeftSonar < SONAR_OBJECT_CLOSE_BOOST && frontLeftSonar != 0) ||
											 (frontRightSonar < SONAR_OBJECT_CLOSE_BOOST && frontRightSonar != 0) :
		(frontLeftSonar < SONAR_OBJECT_CLOSE && frontLeftSonar != 0) ||
		(frontRightSonar < SONAR_OBJECT_CLOSE && frontRightSonar != 0)) {
		if (!rumble) {
			xwii_iface_rumble(iface, true);
			rumble = true;
		}
	} else if (rumble) {
		xwii_iface_rumble(iface, false);
		rumble = false;
	}
}

void leftEncoderCallback(const std_msgs::Int16::ConstPtr &msg) { frontLeftSonar = msg->data; }

void rightEncoderCallback(const std_msgs::Int16::ConstPtr &msg) { frontRightSonar = msg->data; }


int main(int argc, char **argv) {
	ros::init(argc, argv, "wiimote");
	ros::NodeHandle nh;
	signal(SIGINT, mySigintHandler);
	int ret = 0;
	char *path = NULL;
	path = get_dev(1);
	ret = xwii_iface_new(&iface, path);
	free(path);
	if (ret) {
		ROS_ERROR("Cannot create xwii_iface '%s' err:%d\n",
				  argv[1], ret);
		ros::shutdown();
	} else {
		//refresh();

		ret = xwii_iface_open(iface,
							  xwii_iface_available(iface) |
							  XWII_IFACE_WRITABLE);
		if (ret) {
			ROS_ERROR("Error: Cannot open interface: %d", ret);
			//ros::shutdown();
		}
		vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		go_pub = nh.advertise<std_msgs::Bool>("/doFollow", 10);
		sonar_pub = nh.advertise<std_msgs::Bool>("/doSonar", 10);
		vel_msg.linear.x = 0;
		vel_msg.angular.z = 0;
		vel_pub.publish(vel_msg);
		ros::Timer timer = nh.createTimer(ros::Duration(0.01), run_iface);
		updateLEDs();
		timerLED = nh.createTimer(ros::Duration(1), runLEDs, false);
		sonar_subs[1] = nh.subscribe<std_msgs::Int16>("/arduino/sonar_1", 10,
													  rightEncoderCallback);
		sonar_subs[0] = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", 10,
													  leftEncoderCallback);
		ros::spin();
	}

}