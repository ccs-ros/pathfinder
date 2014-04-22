//Includes all the headers necessary to use the most common public pieces of the ROS system.
//#include <iostream>
//#include <stdio.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <computer_vision/Camera.h>
//#include <std_msgs/String.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
/*
void callback(const ros::TimerEvent&)
{
	int num1 = 2;
	int num2 = 3;
	int num3 = num1 + num2; 
	
	timer_test::Result data;
	data.result=num3;
	pub.publish(data);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "timer_test");
    ros::NodeHandle nh;
	
    ros::Publisher pub = nh.advertise<timer_test::Result>("test_topic", 1000); 

    ros::Timer timer1 = nh.createTimer(ros::Duration(0.01), callback);


 
    //pub = it.advertise("usb_cam/image_processed", 1);
	
    ros::spin();

    return 0;
 
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "det_temp_node");
    ros::NodeHandle nh;
	
    ros::Publisher pub = nh.advertise<computer_vision::Camera>("det/cam_select", 1000); 
    ros::Rate loop_rate(10);
    
    int count=1;
    computer_vision::Camera cam_select;

    while(ros::ok())
    {
	if(count==1) cam_select.camera=1;
	else if (count==100) cam_select.camera=2;
	else if (count==200) count=0;
	
	pub.publish(cam_select);
	
	ros::spinOnce();
	loop_rate.sleep();
	++count;
    }

    return 0;
}
