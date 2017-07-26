#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"
#include <csignal>
#include <poll.h>
#include "xwiimote.h"

#define SONAR_OBJECT_CLOSE 35
const double LINEAR_SPEED[4] = {0.5, 0.8, 1.2, 1.5};
const double ANGULAR_SPEED[4] = {0.5, 0.8, 1.2, 1.5};
static struct xwii_iface *iface;
int frontLeftSonar;
int frontRightSonar;
ros::Subscriber sonar_subs[2];
ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;

void mySigintHandler(int sig) {
	xwii_iface_unref(iface);
	ros::shutdown();
}

static char *get_dev(int num) {
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

void changeSpeed(bool increase) {
	if (increase) {
		if (speed < 4) speed++;
	} else if (speed > 1) speed--;
	for (int i = 1; i <= speed; i++) xwii_iface_set_led(iface, XWII_LED(i), true);
	for (int i = 4; i > speed; i--) xwii_iface_set_led(iface, XWII_LED(i), false);
}

static void handle_watch(void) {
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
bool isFirstRun = true;
bool rumble = false;

void run_iface(const ros::TimerEvent &) {
	if (isFirstRun) {
		isFirstRun = false;
		for (int i = 1; i <= speed; i++) xwii_iface_set_led(iface, XWII_LED(i), true);
		for (int i = 4; i > speed; i--) xwii_iface_set_led(iface, XWII_LED(i), false);
	}
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

				if (code == XWII_KEY_LEFT) {
					if (!isKeyDown && pressed) {
						isKeyDown = true;
						keyDownKey = XWII_KEY_LEFT;
						vel_msg.linear.x = 0.9;
						vel_msg.angular.z = ANGULAR_SPEED[speed - 1];
					} else if (keyDownKey == XWII_KEY_LEFT && !pressed) {
						isKeyDown = false;
						vel_msg.linear.x = 0;
						vel_msg.angular.z = 0;
					}
				} else if (code == XWII_KEY_RIGHT) {
					if (!isKeyDown && pressed) {
						isKeyDown = true;
						keyDownKey = XWII_KEY_RIGHT;
						vel_msg.linear.x = 0.9;
						vel_msg.angular.z = -ANGULAR_SPEED[speed - 1];
					} else if (keyDownKey == XWII_KEY_RIGHT && !pressed) {
						isKeyDown = false;
						vel_msg.linear.x = 0;
						vel_msg.angular.z = 0;
					}
				} else if (code == XWII_KEY_UP) {
					if (!isKeyDown && pressed) {
						isKeyDown = true;
						keyDownKey = XWII_KEY_UP;
						vel_msg.linear.x = LINEAR_SPEED[speed - 1];
						vel_msg.angular.z = 0;
					} else if (keyDownKey == XWII_KEY_UP && !pressed) {
						isKeyDown = false;
						vel_msg.linear.x = 0;
						vel_msg.angular.z = 0;
					}
				} else if (code == XWII_KEY_DOWN) {
					if (!isKeyDown && pressed) {
						isKeyDown = true;
						keyDownKey = XWII_KEY_DOWN;
						vel_msg.linear.x = -LINEAR_SPEED[speed - 1];
						vel_msg.angular.z = 0;
					} else if (keyDownKey == XWII_KEY_DOWN && !pressed) {
						isKeyDown = false;
						vel_msg.linear.x = 0;
						vel_msg.angular.z = 0;
					}
				} else if (code == XWII_KEY_MINUS) {
					if (pressed)
						changeSpeed(false);
				} else if (code == XWII_KEY_PLUS) {
					if (pressed)
						changeSpeed(true);
				}
				break;
		}
		vel_pub.publish(vel_msg);
		if (isKeyDown) {
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

			ROS_INFO("Key Down: %s", s.c_str());
		} else
			ROS_INFO("No Key, speed %d, left %d, right %d", speed, frontLeftSonar, frontRightSonar);
	}
	if ((frontLeftSonar < SONAR_OBJECT_CLOSE && frontLeftSonar != 0) ||
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
		vel_msg.linear.x = 0;
		vel_msg.angular.z = 0;
		vel_pub.publish(vel_msg);
		ros::Timer timer = nh.createTimer(ros::Duration(0.01), run_iface);
		sonar_subs[1] = nh.subscribe<std_msgs::Int16>("/arduino/sonar_1", 10,
													  rightEncoderCallback);
		sonar_subs[0] = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", 10,
													  leftEncoderCallback);
		ros::spin();
	}

}