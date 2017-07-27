#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iosfwd>
#include <sstream>
#include <stdlib.h>
#include <std_msgs/UInt64.h>
#include <signal.h>

#define LINEAR_SPEED 0.65
#define ANGULAR_SPEED 1.285

class MotorTest {
public:
    MotorTest();

private:
    ros::Publisher vel_pub;
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;

    void leftEncoderCallback(const std_msgs::UInt64::ConstPtr &msg);

    void rightEncoderCallback(const std_msgs::UInt64::ConstPtr &msg);

    image_transport::Subscriber raw_image_sub;
    unsigned long long leftEncoder;
    unsigned long long rightEncoder;
    ros::Subscriber left_encoder_sub;
    ros::Subscriber right_encoder_sub;
    bool isTurning = true;
    int diameter = 2;
    ros::WallTimer timer;


    void cameraUpdate(const ros::WallTimerEvent &);

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    bool takePicture = false;
};

MotorTest::MotorTest() {
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    left_encoder_sub = nh.subscribe<std_msgs::UInt64>("/arduino/encoder_left_value", 10, &MotorTest::leftEncoderCallback,
                                                     this);
    right_encoder_sub = nh.subscribe<std_msgs::UInt64>("/arduino/encoder_right_value", 10,
                                                      &MotorTest::rightEncoderCallback, this);
    image_transport::ImageTransport it(nh);
    raw_image_sub = it.subscribe("/usb_cam/image_raw", 1, &MotorTest::imageCallback,this);
    timer = nh.createWallTimer(ros::WallDuration(0.2), &MotorTest::cameraUpdate, this);
}

void MotorTest::cameraUpdate(const ros::WallTimerEvent &) {
    if (isTurning) {
        takePicture = true;
    }


}
//2D: 9373, 5781
//5D: 11261, 15022
//2D: 11, 7
//5D: 14, 18
//2D: 11, 7
//5D: 132.72, 170.64
//2D: 11, 7
//5D: 42, 54
//2D: 11, 7
//5D: 3, 4
void MotorTest::leftEncoderCallback(const std_msgs::UInt64::ConstPtr &msg) {
    leftEncoder = msg->data;
}
void MotorTest::rightEncoderCallback(const std_msgs::UInt64::ConstPtr &msg) {
    rightEncoder = msg->data;
    if (isTurning) {

        vel_msg.linear.x = LINEAR_SPEED;
        vel_msg.angular.z = ANGULAR_SPEED / diameter;
        vel_pub.publish(vel_msg);
        ROS_INFO("Left Encoder: %u, Right Encoder: %u",leftEncoder,rightEncoder);
    }
}

void MotorTest::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        if (takePicture) {
            cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::stringstream s;
            s << "/home/ubuntu/Pictures/Picture" << time(0) << ".jpg";
            cv::imwrite(s.str().c_str(), src);
            takePicture = false;
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_test");

    MotorTest motor_test;
    ros::spin();
}