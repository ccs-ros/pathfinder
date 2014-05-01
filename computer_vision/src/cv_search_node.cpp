#include <ros/ros.h> 							//includes headers common for ros system
#include <image_transport/image_transport.h> 	//publush and subscribe to ros images
#include <cv_bridge/cv_bridge.h> 				//convert between ros and opencv image formats
#include <sensor_msgs/image_encodings.h> 		//constants for image encoding
#include <path_planning/State.h>				//message containing mission state
#include <computer_vision/Search.h>				//message containing sample and beacon structure
#include <search.hpp>							//custom header containing search functions

#define PI 3.14159265359

using namespace std;
namespace enc = sensor_msgs::image_encodings;

//Debugging
const bool HZ = 0; //Print sampling rate

//Show Images
const int ORIG_IMAGE = 1;
const int THRE_IMAGE = 1;
const int HUE_TRACK = 1;

//Topic Names
static const char IMAGE_TOPIC[] = "logitech_c920/image_raw";
static const char STATE_TOPIC[] = "mis/state";
static const char SEARCH_TOPIC[] = "/det/search_data";

class State //Class containing callback function for data coming into node
{
	public:
	ros::Subscriber sub;

	uint8_t state, vision_state;

	State() : state(0), vision_state(3)
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

//Global
ros::Publisher pub;
computer_vision::Search outData;

//Function headers
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image);

//MAIN
int main(int argc, char **argv)
{
   ros::init(argc, argv, "cv_search_node");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
    
   cv::namedWindow("Origional Image", CV_WINDOW_AUTOSIZE);
   cv::destroyWindow("Origional Image");

   image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
   pub = nh.advertise<computer_vision::Search>(SEARCH_TOPIC, 1);

   double time_curr = ros::Time::now().toSec();
   double time_prev = time_curr;
   
   ROS_INFO("Running cv_search_node...");
   ros::spin();
   
   return 0;
}

//CALLBACK
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
	double begin = ros::Time::now().toSec(); //start timer
	
	//structures for subscribing and publishing
	cv_bridge::CvImagePtr cv_ptr;
	State inData; //vision_state 1 pre-cached, 2 sample, 3 homing

	//Initialize outputs
	outData.beacon_seen=0;
	outData.sample_seen=0;
	outData.landmark_seen=0;
	
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
     
    if(inData.vision_state==1) //pre-cached sample search
    {
    	TrackingObject PreCach("Pre-Cached");
    	vector <TrackingObject> PreCachs;
    	
    	cv::Mat thresh = filterWhite(cv_ptr->image, 220, 255, 3, 3, 5, THRE_IMAGE); //frame, min_thresh, max_thresh, thresh_type, erode, dilate, filter, show thresh
    	PreCachs = findCandidates(cv_ptr->image, thresh, PreCach.getType(), 50, 8*8, 400*400, HUE_TRACK); //frame, thresh, type, max_obj, min_area, max_area, show hue track
    	
    	if(PreCachs.size()>0)
    	{
    		outData.sample_seen=1;
    		outData.sample_type=0;
		outData.sample_x=PreCachs[0].getxPos();
		outData.sample_y=PreCachs[0].getyPos();
		outData.sample_area=PreCachs[0].getarea();
    	}
    }
    else if (inData.vision_state==2) //sample search
    {
    	//sample strucutre
    	TrackingObject sample("Red Sample");
    	vector <TrackingObject> samples;
    	
    	//find hues
		cv::Mat thresh = filterColors(cv_ptr->image, sample.getHSVmin(), sample.getHSVmax(), 3, 5, 9, THRE_IMAGE); //frame, type, erode, dilate, filter, show thresh
		samples = findCandidates(cv_ptr->image, thresh, sample.getType(), 50, 8*8, 400*400, HUE_TRACK); //frame, thresh, type, max_obj, min_area, max_area, show hue track
    }
    else if(inData.vision_state==3) //homing search
    {
    	//red object structure
    	TrackingObject redBall("Red Ball");
    	vector <TrackingObject> redBalls; 
    	
    	//find red objects
		cv::Mat thresh = filterColors(cv_ptr->image, redBall.getHSVmin(), redBall.getHSVmax(), 3, 5, 9, THRE_IMAGE); //frame, type, erode, dilate, filter, show thresh
		redBalls = findCandidates(cv_ptr->image, thresh, redBall.getType(), 50, 8*8, 400*400, HUE_TRACK); //frame, thresh, type, max_obj, min_area, max_area, show hue track
		
		if(redBalls.size()>=1) //Found at least three red objects
		{
			//blue object structure
			TrackingObject SideOne("Red Ball");
			vector <TrackingObject> SideOnes;
			
    		//find blue object
			cv::Mat thresh = filterColors(cv_ptr->image, SideOne.getHSVmin(), SideOne.getHSVmax(), 3, 5, 9, THRE_IMAGE); //frame, type, erode, dilate, filter, show thresh
			SideOnes = findCandidates(cv_ptr->image, thresh, SideOne.getType(), 50, 8*8, 400*400, HUE_TRACK); //frame, thresh, type, max_obj, min_area, max_area, show hue track
			
			if(SideOnes.size()>=1) //Found at least one blue object
			{
				if(SideOnes.size()==3)
				{		
					float B = findHomingBearing(SideOnes, 3.67, 640, 480); //objects, focal_length
					cout << "bearing = " << 180/PI*B << endl;
				}
				else ROS_WARN("Need exactly 3 objects to calculate bearing!");

				outData.beacon_seen=1;
				outData.beacon_x=SideOnes[0].getxPos();
				outData.beacon_y=SideOnes[0].getyPos();
				outData.beacon_area=SideOnes[0].getarea();
			}
		}
    }
    else if(inData.vision_state==4) //landmark search
    {
    }
    else //no state selected
    {
    	ROS_WARN("No vision_state specified or out of range in computer_vision::cv_search_node!");
    }
    
    pub.publish(outData); //publish search results
    
	if(ORIG_IMAGE) cv::imshow("Origional Image", cv_ptr->image); cv::waitKey(1);
    
	double end = ros::Time::now().toSec(); //stop timer
	if(HZ) ROS_INFO("Sample Rate = %f", 1/(end-begin)); //print execution time
}
