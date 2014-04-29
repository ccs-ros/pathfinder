#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <path_planning/Avoidance.h>
#include <vector>

using namespace std;

static const char LASERSCAN_TOPIC[] = "/scan";
static const char AVOIDANCE_TOPIC[] = "mis/avoidance";

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
	ros::init(argc, argv, "nav_object_avoidance_node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<path_planning::Avoidance>(AVOIDANCE_TOPIC,1);

	Scan inData; //structure for input data
	path_planning::Avoidance outData; //structure for current state

	while(inData.scan_time==0) 
	{
		ROS_INFO("Waiting for first measurement...");
		ros::Duration(0.02).sleep();
		ros::spinOnce();
	}

	ROS_INFO("First measurement received! ");
	ROS_INFO("nav_object_avoidance_node running...");

	while(ros::ok())
	{	
		outData.avoidance_state=1;
		//pub.publish(outData);
		ros::spinOnce();
		ros::Duration(0.02).sleep(); 
	}
	
	return 0;
}
