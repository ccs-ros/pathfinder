//Publishes Test Data for Navigation Node

#include <ros/ros.h>
#include <navigation/FilterInput.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_temp_node");
    ros::NodeHandle nh;
	
    ros::Publisher pub = nh.advertise<navigation::FilterInput>("nav/temp_data", 1); 
    ros::Rate loop_rate(50);
    
    ROS_INFO("nav_temp_node started...");
    
    int count = 0;
    navigation::FilterInput temp_data;

    while(ros::ok())
    {
    	temp_data.encFL = 0.02*1.1;
    	temp_data.encFR = 0.02*0.9;
    	temp_data.encBL = 0.02*1.1;
    	temp_data.encBR = 0.02*0.9;
   	temp_data.p = 0;
    	temp_data.q = 0;
    	temp_data.r = 0;
    	temp_data.ax = 0;
    	temp_data.ay = 0;
    	temp_data.az = 9.81;
    	temp_data.stopFlag = 0;
    	temp_data.turnFlag = 0;
    	
	pub.publish(temp_data);
		
	ros::spinOnce();
	loop_rate.sleep();
	++count;
    }

    return 0;
}
