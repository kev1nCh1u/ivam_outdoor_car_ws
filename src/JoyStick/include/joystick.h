#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <linux/joystick.h>
#include <vector>
#include <unistd.h>
#include <boost/thread.hpp>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "JoyStick/joystick.h"
#include <JoyStick/Node_recv.h>

using namespace std;


#define PUSE_BUTTON_A 0
#define PUSE_BUTTON_B 1
#define PUSE_BUTTON_X 2
#define PUSE_BUTTON_Y 3
#define PUSE_BUTTON_ID 4
#define PUSE_BUTTON_RB 5
#define PUSE_BUTTON_BACK 6
#define PUSE_BUTTON_START 7

#define PUSE_BUTTON_LT 11
#define PUSE_BUTTON_RT 12

struct joystick_position {
    float theta, r, x, y;
};

struct joystick_state {
    vector<signed short> button;
    vector<signed short> axis;
};

class Joystick
{
public:
    Joystick(char *joystick_dev);
    ~Joystick();
    void readEv();
    void RevProcess(double receive_period);
    joystick_position joystickPosition(int n);
    bool buttonPressed(int n);
    bool buttonPressed_LRT(int n);
    bool checkPause();
    ssize_t recvtimeout(int ss, js_event *buf, int timeout);
    
    bool active;
private:
   pthread_t thread;
   
   int joystick_fd;
   js_event *joystick_ev;
   joystick_state *joystick_st;
   __u32 version;
   __u8 axes;
   __u8 buttons;
   char name[256];

   bool isPause;

   int btn_id;
   int btn_last;

   boost::thread* receive_thread_;

   ros::NodeHandle nodeHandle_;

   ros::Publisher Point_data_publisher_;
   ros::Publisher IdType_Publisher_;

   geometry_msgs::Point Point_data_;
   
   joystick_position command_pos;

};
struct NODE_recv{

	int id;
	int type;
	std::string kin;
	float id_pixel_x;
	float id_pixel_y;
	float id_heading;
	int time;
	bool btn_finish;
	int line;
	float radius;

	Eigen::Vector3f node_pose;
};

#endif // JOYSTICK_H


