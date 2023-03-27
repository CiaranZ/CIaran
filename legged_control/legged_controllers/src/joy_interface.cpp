
#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <Eigen/Core>
// #include "pinocchio/math/matrix.hpp"

static sensor_msgs::Joy joytemp;
void statecmdcallback(sensor_msgs::Joy cmd)
{
  joytemp =cmd;
} 

int main (int argc, char** argv){
	//创建ros节点jason
     ros::init(argc, argv, "XBOX");
     ros::NodeHandle nh;
    //创建三个发布者
    ros::Publisher xboxpub=nh.advertise<sensor_msgs::Joy>("/xboxvcmd",1);
    ros::Subscriber xboxsub = nh.subscribe("/joy", 1, statecmdcallback);
    std::vector<float> temp1(8);
    std::vector<int>  temp2(15);
   //  temp1.setZero();
   //  temp2.setZero();
    joytemp.axes = temp1;
    joytemp.buttons = temp2;
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
      xboxpub.publish(joytemp);
      ros::spinOnce(); 
      loop_rate.sleep();
    }
}