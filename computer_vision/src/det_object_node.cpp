#include <ros/ros.h> 								//includes headers common for ros system
#include <vector> 									//hue detection
#include <sstream> 									//hue detection
#include <armadillo>								//linear algebra library
#include <image_transport/image_transport.h> 		//publush and subscribe to ros images
#include <cv_bridge/cv_bridge.h> 					//convert between ros and opencv image formats
#include <sensor_msgs/image_encodings.h> 			//constants for image encoding
#include <opencv2/imgproc/imgproc.hpp> 				//image processing
#include <opencv2/highgui/highgui.hpp> 				//gui handling
#include <opencv2/core/core.hpp>					//core functions and data types
#include <opencv2/features2d/features2d.hpp>		//2d features framework
#include <opencv2/nonfree/features2d.hpp>			//SURF algorithms
#include <opencv2/calib3d/calib3d.hpp>				//camera calibration and 3d reconstruction
#include <opencv2/legacy/legacy.hpp>				//brute force matching
#include <computer_vision/Beacon.h>					//message for homing beacon

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

//Show images hue
const bool SHOW_ORIGIONAL_IMAGE = false;	//image callback	
const bool SHOW_THRESH_IMAGE = true;		//filterHue function
const bool DRAW_HUE_TRACKING = true;		//findCandidates function 
const bool DRAW_CIRCLES = true; 			//hough circle function

//Show images features
const bool DRAW_FOUND_MATCHES = true;

//Debugging
const bool CALIBRATE_MODE = false;
const bool HZ = false; //print tracking frequency
const bool DEBUG_HOMOGRAPHY = false;
const bool PRINT_POSE = false;
const bool ORB_VERIFICATION = true;

//Topic Names
static const char IMAGE_TOPIC[] = "logitech_c920/image_raw";
static const char OUTPUT_DATA_TOPIC[] = "det/mis_out_data";

//File locations
static const string home = getenv("HOME");
static const string BEACON_IMG_ADDRESS = home + "/catkin_ws/src/computer_vision/images/beacon4.png";

//Global variables
ros::Publisher pub;

int H_MIN = 161;
int S_MIN = 89;
int V_MIN = 79;
int H_MAX = 180;
int S_MAX = 181;
int V_MAX = 244;

int c1_;
double area_;

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS=50;
const int MIN_OBJECT_AREA = 1*1;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}

void morphOps(Mat &thresh){

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);

}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
	 if(hierarchy.size()==0){
		 c1_=0;
		 cout<<c1_<<endl;
	 }
		int numObjects = hierarchy.size();
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				area_ = moment.m00;
                if(area_>MIN_OBJECT_AREA && area_<MAX_OBJECT_AREA && area_>refArea){
					x = moment.m10/area_;
					y = moment.m01/area_;
					objectFound = true;
					refArea = area_;
				}else objectFound = false;


			}
			if(objectFound ==true){
				c1_=1;
				cout<<c1_<<"  x=  "<<x<<"  y=  "<<y<<"  area=  "<<area_<<endl;
			}else{
				c1_=0;
				cout<<c1_<<endl;
		}
	  }
   }else {
	   c1_=0;
	   cout<<c1_<<endl;
	}
}


bool trackObjects = true;
bool useMorphOps = true;
Mat HSV;
Mat Threshold;
int x=0, y=0;

void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    double begin = ros::Time::now().toSec(); //start timer
    
    //structures for subscribing and publishing
    cv_bridge::CvImagePtr cv_ptr;
    
    //Output variable
    computer_vision::Beacon outData;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("computer_vision::det_hue_node.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    
    Mat cameraFeed = cv_ptr->image;
    
    cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
    
	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),Threshold);
	
	if(useMorphOps) morphOps(Threshold);
	
	if(trackObjects) trackFilteredObject(x,y,Threshold,cameraFeed);
    
    if(SHOW_ORIGIONAL_IMAGE==true) 
    {
		cv::imshow("Origional Image (det_hue_node)", cv_ptr->image);
		cv::waitKey(1);
    }

    outData.centerx = x;
    outData.centery = y;
    outData.areaobject = area_;
    outData.object = c1_;
    
    pub.publish(outData);
    
    double end = ros::Time::now().toSec(); //stop timer
    if(HZ) ROS_INFO("Sample Rate = %f", 1/(end-begin)); //print execution time
}

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

