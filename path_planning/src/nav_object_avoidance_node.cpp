// Created on 4/21/14

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

using namespace std;

static const char LASERSCAN_TOPIC[] = "/scan"; 

class Scan //Class containing callback function for data coming into node
{
	public:
  		ros::Subscriber sub1;
  		float scan_time, range_min, range_max, angle_increment, time_increment, scan_size;
  		float ranges[], intensities[];

  		Scan() : scan_time(0), range_min(0), range_max(0), angle_increment(0), time_increment(0)
  		{
    		ros::NodeHandle node;
    		sub1 = node.subscribe(LASERSCAN_TOPIC, 1, &Scan::getScanCallback, this);
  		}

  		void getScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) 
		{
    		this->scan_time = msg->scan_time;
    		this->range_min = msg->range_min;
    		this->range_max = msg->range_max;
    		this->angle_increment = msg->angle_increment;
    		this->time_increment = msg->time_increment;
    		this->scan_size = msg->ranges.size();
    
    		for (int i=0; i<msg->ranges.size(); i++) this->ranges[i] = msg->ranges[i];
    		for (int i=0; i<msg->intensities.size(); i++) this->intensities[i] = msg->intensities[i];
  		}
};

int main(int argc, char **argv)
{
	//Node Initialization
	ros::init(argc, argv, "nav_laserscan_node");
	ros::NodeHandle nh;
    
	ROS_INFO("nav_filter_node running...");
    
	Scan inData;	//structure for input data
    
	while(inData.scan_time==0) 
	{
		ROS_INFO("Waiting for first measurement...");
		ros::spinOnce();
	}

	while(ros::ok())
	{	
		ros::spinOnce();
		ros::Duration(0.05).sleep(); //20HZ
	}
	
	return 0;
}
