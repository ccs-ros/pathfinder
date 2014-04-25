#include <ros/ros.h> 							//includes headers common for ros system
#include <image_transport/image_transport.h> 	//publush and subscribe to ros images
#include <cv_bridge/cv_bridge.h> 				//convert between ros and opencv image formats
#include <sensor_msgs/image_encodings.h> 		//constants for image encoding
#include <computer_vision/Beacon.h>				//message for homing beacon
#include <path_planning/State.h>
#include <search.hpp>							//custom header containing search functions

using namespace std;
namespace enc = sensor_msgs::image_encodings;

//Debugging
const bool HZ = 1; //Print sampling rate

//Show Images
const int ORIG_IMAGE = 1;
const int THRE_IMAGE = 1;
const int HUE_TRACK = 1;

//Topic Names
static const char IMAGE_TOPIC[] = "logitech_c920/image_raw";
static const char STATE_TOPIC[] = "mis/state";

//Publishers
ros::Publisher pub;

class State //Class containing callback function for data coming into node
{
	public:
	ros::Subscriber sub;

	uint8_t state, vision_state;

	State() : state(0), vision_state(0)
	{
    	ros::NodeHandle node;
    	sub = node.subscribe(STATE_TOPIC, 1, &State::getData1Callback, this);
	}

	void getData1Callback(const path_planning::State::ConstPtr &msg) 
	{
		this->state = msg->state;
		this->vision_state = msg->vision_state;
	}
};

//Callback function
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image);

int main(int argc, char **argv)
{
   ros::init(argc, argv, "cv_search_node");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
    
   cv::namedWindow("Origional Image", CV_WINDOW_AUTOSIZE);
   cv::destroyWindow("Origional Image");

   image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);

   double time_curr = ros::Time::now().toSec();
   double time_prev = time_curr;
   
   ROS_INFO("Running cv_search_node...");
   ros::spin();
   
   return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
	double begin = ros::Time::now().toSec(); //start timer
	
	//structures for subscribing and publishing
	cv_bridge::CvImagePtr cv_ptr;
	State inData; //vision_state 1 search, 2 homing
	
	//get image from camera node
	try
	{
        cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
        ROS_ERROR("computer_vision::cv_search_node.cpp::cv_bridge exception: %s", e.what());
        return;
	}
    
    //object structures
    TrackingObject pinkball("Pink Sample");
    vector <TrackingObject> pinkballs;
    
    if(inData.vision_state==1) //search state
    {
    	cv::Mat thresh = filterColors(cv_ptr->image, pinkball.getHSVmin(), pinkball.getHSVmax(), 3, 5, 9, THRE_IMAGE); //frame, type, erode, dilate, filter, show thresh
    	pinkballs = findCandidates(cv_ptr->image, thresh, "Pink Sample", 50, 8*8, 400*400, HUE_TRACK); //frame, thresh, type, max_obj, min_area, max_area, show hue track
    }
    else if (inData.vision_state==2) //vision state
    {
    	cv::Mat thresh = filterWhite(cv_ptr->image, 220, 255, 3, 3, 5, THRE_IMAGE); //frame, min_thresh, max_thresh, thresh_type, erode, dilate, filter, show thresh
    }
    else
    {
    	ROS_WARN("No vision_state specified!");
    }
    
	cv::imshow("Origional Image", cv_ptr->image);
	cv::waitKey(1);
    
	double end = ros::Time::now().toSec(); //stop timer
	if(HZ) ROS_INFO("Sample Rate = %f", 1/(end-begin)); //print execution time
}
