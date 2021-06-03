#include <ros/ros.h>
#include <serial.serial.h>
#include <std_msgs/String.h>
#include <iosstream>
#include "imu_data_decaode.h"
#include "package.h"


#define IMU_SERIAL "/dev/ttyUSB0"
#define BAUD (115200)

int main(int argc,char )
{
    ros::init(argc,argv,"serialIMU");
    ros::NodeHandle node;
    ros::Publisher IMU_pub = node.advertise<std_msgs::String>("chatter",1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        
    }
    
}