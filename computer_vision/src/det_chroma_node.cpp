#include <ros/ros.h> 						//includes headers common for ros system
#include <vector> 							//hue detection
#include <sstream> 							//hue detection
#include <armadillo>						//linear algebra library
#include <image_transport/image_transport.h> 	//publush and subscribe to ros images
#include <cv_bridge/cv_bridge.h> 				//convert between ros and opencv image formats
#include <sensor_msgs/image_encodings.h> 		//constants for image encoding
#include <opencv2/imgproc/imgproc.hpp> 			//image processing
#include <opencv2/highgui/highgui.hpp> 			//gui handling
#include <opencv2/core/core.hpp>				//core functions and data types
#include <opencv2/features2d/features2d.hpp>		//2d features framework
#include <opencv2/nonfree/features2d.hpp>		//SURF algorithms
#include <opencv2/calib3d/calib3d.hpp>			//camera calibration and 3d reconstruction
#include <opencv2/legacy/legacy.hpp>			//brute force matching
#include <computer_vision/Beacon.h>			//message for homing beacon

using namespace std;
namespace enc = sensor_msgs::image_encodings;

//Debugging
const bool SHOW_ORIGIONAL_IMAGE = true;		//image callback	
const bool SHOW_THRESH_IMAGE = false;		//filterHue function
const bool DRAW_HUE_TRACKING = false;		//findCandidates function 
const bool CALIBRATE_MODE = false;			//trackbars for calibration
const bool HZ = true; 					//print tracking frequency

//Topic Names
static const char IMAGE_TOPIC[] = "logitech_c920/image_raw";
static const char OUTPUT_DATA_TOPIC[] = "det/mis_out_data";

//Global variables for surf
ros::Publisher pub;

//Callback function
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image);


int main(int argc, char **argv)
{
   ros::init(argc, argv, "det_hue_node");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
    
   cv::namedWindow("Origional Image (det_hue_node)", CV_WINDOW_AUTOSIZE);
   cv::destroyWindow("Origional Image (det_hue_node)");

   image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
   pub = nh.advertise<computer_vision::Beacon>(OUTPUT_DATA_TOPIC, 1);

   double time_curr = ros::Time::now().toSec();
   double time_prev = time_curr;
   
   ROS_INFO("Running det_hue_node...");
   ros::spin();
   
   return 0;
}


void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    double begin = ros::Time::now().toSec(); //start timer
    
    //structures for subscribing and publishing
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("computer_vision::det_hue_node.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    
    if(SHOW_ORIGIONAL_IMAGE==true) 
    {
		cv::imshow("Origional Image (det_hue_node)", cv_ptr->image);
		cv::waitKey(1);
    }
    
    double end = ros::Time::now().toSec(); //stop timer
    if(HZ) ROS_INFO("Sample Rate = %f", 1/(end-begin)); //print execution time
}
