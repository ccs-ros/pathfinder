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
const bool SHOW_THRESH_IMAGE = true;		//filterHue function
const bool DRAW_HUE_TRACKING = true;		//findCandidates function 
const bool CALIBRATE_MODE = true;			//trackbars for calibration
const bool HZ = true; 					//print tracking frequency

//Topic Names
static const char IMAGE_TOPIC[] = "logitech_c920/image_raw";
static const char OUTPUT_DATA_TOPIC[] = "det/mis_out_data";

//Global variables for surf
ros::Publisher pub;

//Max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;

//Minimum and maximum object hue area
const int MIN_OBJECT_AREA = 1 * 1;
const int MAX_OBJECT_AREA = 400 * 400;

//Class for homing beacon tracking
class TrackingObject
{
	public:
	TrackingObject(void){};
	~TrackingObject(void){};

	TrackingObject(std::string name)
	{
		setType(name);
		
		if(name ==  "SideOne")
		{

			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(180, 256, 256));
			setColor(cv::Scalar(255, 0, 0));
		}

		if(name ==  "SideTwo")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(180, 256, 256));
			setColor(cv::Scalar(0, 0, 255));
		}
		
		if(name == "Cropped")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(180, 256, 256));
			setColor(cv::Scalar(0, 0, 255));		
		}
		
		if(name == "Segment")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(180, 256, 256));
			setColor(cv::Scalar(0, 0, 255));		
		}
	}

	int getxPos() {return xPos;}
	void setxPos(int x) {xPos=x;}

	int getyPos() {return yPos;}
	void setyPos(int y) {yPos=y;};
	
	int getarea() {return areaObj;}
	void setarea(int area) {areaObj=area;};

	cv::Scalar getHSVmin() {return HSVmin;}
	cv::Scalar getHSVmax() {return HSVmax;}

	void setHSVmin(cv::Scalar min) {HSVmin=min;}
	void setHSVmax(cv::Scalar max) {HSVmax=max;}

	std::string getType() {return type;}
	void setType(std::string t) {type = t;}

	cv::Scalar getColor() {return Color;}
	void setColor(cv::Scalar c) {Color = c;}
	
	private:
	int xPos, yPos, area, areaObj;
	std::string type;
	cv::Scalar HSVmin, HSVmax;
	cv::Scalar Color;
};

//Callback function
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image);

//Functions
void filterHSV(cv::Mat &frame, const int &COLOR);
void filterYUV(cv::Mat &frame, const int &COLOR);
void morphImg(cv::Mat &thresh);

void createTrackbarsYUV();
void createTrackbarsHSV();
void on_trackbarHSV(int, void*){} 
void on_trackbarYUV(int, void*){}

vector <TrackingObject> findCandidates(const cv::Mat &frame, const cv::Mat imgThresh);

void drawHueDetection(const cv::Mat &frame, vector <TrackingObject> &Segments);
string intToString(int number);

//HSV YUV
int H_MIN = 0;
int S_MIN = 0;
int Vh_MIN = 0;
int H_MAX = 255;
int S_MAX = 255;
int Vh_MAX = 255;

int Y_MIN = 0;
int U_MIN = 0;
int Vy_MIN = 0;
int Y_MAX = 255;
int U_MAX = 255;
int Vy_MAX = 255;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "det_chroma_node");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
    
   cv::namedWindow("Origional Image", CV_WINDOW_AUTOSIZE);
   cv::destroyWindow("Origional Image");

   image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
   pub = nh.advertise<computer_vision::Beacon>(OUTPUT_DATA_TOPIC, 1);

   double time_curr = ros::Time::now().toSec();
   double time_prev = time_curr;
   
   ROS_INFO("Running det_chroma_node...");
   ros::spin();
   
   return 0;
}


void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
	double begin = ros::Time::now().toSec(); //start timer
    
	//structures for subscribing and publishing
	cv_bridge::CvImagePtr cv_ptr;

	//Image variables
	vector <TrackingObject> Segments;

	try
	{
        cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
        ROS_ERROR("computer_vision::det_chroma_node.cpp::cv_bridge exception: %s", e.what());
        return;
	}
    
	if(SHOW_ORIGIONAL_IMAGE==true) 
	{
		cv::imshow("Origional Image", cv_ptr->image);
		cv::waitKey(1);
	}

	cv::Mat imgThreshHSV = cv_ptr->image;
	cv::Mat imgThreshYUV = cv_ptr->image;
	filterHSV(imgThreshHSV, 0);
	filterYUV(imgThreshYUV, 0);

	//Segments = findCandidates(cv_ptr->image, imgThreshHSV);
    
	double end = ros::Time::now().toSec(); //stop timer
	if(HZ) ROS_INFO("Sample Rate = %f", 1/(end-begin)); //print execution time
}

//Converts given image to binary image of threshold values
void filterHSV(cv::Mat &frame, const int &COLOR)
{
	if(CALIBRATE_MODE==true) createTrackbarsHSV(); 
	else
	{
		H_MIN = 161;
		S_MIN = 89;
		Vh_MIN = 79;
		H_MAX = 180;
		S_MAX = 181;
		Vh_MAX = 244;
	}

	//Image variables
	cv::Mat imgHSV, imgThresh;
	cv::Mat img = frame;

	//Process source image
	cv::GaussianBlur(frame, img, cv::Size(9,9), 0); //(41,41), (11,11)
	cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

	cv::inRange(imgHSV, cv::Scalar(H_MIN, S_MIN, Vh_MIN), cv::Scalar(H_MAX, S_MAX, Vh_MAX), imgThresh); 

	morphImg(imgThresh);			

	frame = imgThresh;

	if(SHOW_THRESH_IMAGE) cv::namedWindow("Threshold Image HSV");
	if(SHOW_THRESH_IMAGE) cv::imshow("Threshold Image HSV", imgThresh); cv::waitKey(1);
}

//Converts given image to binary image of threshold values
void filterYUV(cv::Mat &frame, const int &COLOR)
{
	if(CALIBRATE_MODE==true) createTrackbarsYUV(); 
			
	//Image variables
	cv::Mat imgYUV, imgThresh;
	cv::Mat img = frame;
    
	//Process source image
	cv::GaussianBlur(frame, img, cv::Size(9,9), 0); //(41,41), (11,11)
	cv::cvtColor(img, imgYUV, cv::COLOR_BGR2YUV);
    
	cv::inRange(imgYUV, cv::Scalar(Y_MIN, U_MIN, Vy_MIN), cv::Scalar(Y_MAX, U_MAX, Vy_MAX), imgThresh); 

	morphImg(imgThresh);											
    
	frame = imgThresh;
    
	if(SHOW_THRESH_IMAGE) cv::namedWindow("Threshold Image YUV");
	if(SHOW_THRESH_IMAGE) cv::imshow("Threshold Image YUV", imgThresh); cv::waitKey(1);
}

//Trackbars for calibration HSV
void createTrackbarsHSV()
{
	cv::namedWindow("Trackbars HSV", 0);

	char TrackbarNameHSV[50];
	sprintf(TrackbarNameHSV, "H_MIN", H_MIN);
	sprintf(TrackbarNameHSV, "H_MAX", H_MAX);
	sprintf(TrackbarNameHSV, "S_MIN", S_MIN);
	sprintf(TrackbarNameHSV, "S_MAX", S_MAX);
	sprintf(TrackbarNameHSV, "Vh_MIN", Vh_MIN);
	sprintf(TrackbarNameHSV, "Vh_MAX", Vh_MAX);
  
	cv::createTrackbar("H_MIN", "Trackbars HSV", &H_MIN, H_MAX, on_trackbarHSV);
	cv::createTrackbar("H_MAX", "Trackbars HSV", &H_MAX, H_MAX, on_trackbarHSV);
	cv::createTrackbar("S_MIN", "Trackbars HSV", &S_MIN, S_MAX, on_trackbarHSV);
	cv::createTrackbar("S_MAX", "Trackbars HSV", &S_MAX, S_MAX, on_trackbarHSV);
	cv::createTrackbar("Vh_MIN", "Trackbars HSV", &Vh_MIN, Vh_MAX, on_trackbarHSV);
	cv::createTrackbar("Vh_MAX", "Trackbars HSV", &Vh_MAX, Vh_MAX, on_trackbarHSV);
}

//Trackbars for calibration HSV
void createTrackbarsYUV()
{
	cv::namedWindow("Trackbars YUV", 0);

	char TrackbarNameYUV[50];
	sprintf(TrackbarNameYUV, "Y_MIN", Y_MIN);
	sprintf(TrackbarNameYUV, "Y_MAX", Y_MAX);
	sprintf(TrackbarNameYUV, "U_MIN", U_MIN);
	sprintf(TrackbarNameYUV, "U_MAX", U_MAX);
	sprintf(TrackbarNameYUV, "Vy_MIN", Vy_MIN);
	sprintf(TrackbarNameYUV, "Vy_MAX", Vy_MAX);

	cv::createTrackbar("Y_MIN", "Trackbars YUV", &Y_MIN, Y_MAX, on_trackbarYUV);
	cv::createTrackbar("Y_MAX", "Trackbars YUV", &Y_MAX, Y_MAX, on_trackbarYUV);
	cv::createTrackbar("U_MIN", "Trackbars YUV", &U_MIN, U_MAX, on_trackbarYUV);
	cv::createTrackbar("U_MAX", "Trackbars YUV", &U_MAX, U_MAX, on_trackbarYUV);
	cv::createTrackbar("Vy_MIN", "Trackbars YUV", &Vy_MIN, Vy_MAX, on_trackbarYUV);
	cv::createTrackbar("Vy_MAX", "Trackbars YUV", &Vy_MAX, Vy_MAX, on_trackbarYUV);
}

//Erodes and dilates image for tracking
void morphImg(cv::Mat &thresh)
{
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

// Returns vector of class TrackingObject of segments constaining circles
vector <TrackingObject> findCandidates(const cv::Mat &frame, const cv::Mat imgThresh)
{
	//Copy frame
	cv::Mat img = frame;
	
	//Object variables
	vector <TrackingObject> Segments;	//vector of candidates
	TrackingObject Segment("Segment");	//candidate of interest
	
	//Contour variables
    vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
    
    //Find contours in threshold image
    cv::findContours(imgThresh, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//Bounding Box variables
	vector<cv::Rect> boundRect(contours.size());
    vector<vector<cv::Point> > contours_poly(contours.size());
    int numberCircles = 0;

	//If not too noisy and not empty
	if(hierarchy.size() < MAX_NUM_OBJECTS && hierarchy.size()>0) 
	{
		//Loop through each contour
		for(int i = 0; i>=0; i=hierarchy[i][0])
		{
			//Moment for particular contour
			cv::Moments moment = moments((cv::Mat)contours[i]);
    		//If object with size limites
			if((moment.m00>MIN_OBJECT_AREA) && (moment.m00<MAX_OBJECT_AREA))
			{
				//Crop image using bounding box
				cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
				boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

					cv::rectangle(img, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255,0,255), 2, 8, 0 );
				
					cv::Mat croppedImage = img(boundRect[i]).clone();
					//cout << boundRect[i].tl() << endl;
    
    				//Find if cropped contains circle
    				//numberCircles = findHoughCircles(croppedImage);

    			//If image contains circle save coordinates
    			//if(numberCircles>0)
    			//{
    				//cout << "circles " << numberCircles << endl;
					Segment.setxPos(moment.m10/moment.m00);//x_=moment.m10/area_;
					Segment.setyPos(moment.m01/moment.m00);//y_=moment.m01/area_;
					Segment.setarea(moment.m00);
					Segment.setType(Segment.getType());
					Segment.setColor(Segment.getColor());
    				
					Segments.push_back(Segment);
				//}
    				//cv::Mat H = trackHomingBeacon(croppedImage);
    			//if(DRAW_HUE_TRACKING==true) drawHueDetection(frame, Segments);				
    		}
    	}
    }
	
	drawHueDetection(frame, Segments);
	if(DRAW_HUE_TRACKING == true) imshow("Hue Tracking", img); cv::waitKey(1);
	return Segments;
}

string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//Draws detected hues from TrackingObject class in given frame
void drawHueDetection(const cv::Mat &frame, vector <TrackingObject> &Segments)
{
	cv::Mat img = frame;
	
	for (int i = 0; i < Segments.size(); i++)
	{
		Segments.at(i).getxPos();
		Segments.at(i).getyPos();
		Segments.at(i).getarea();

		cv::circle(img, cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos()), 10, cv::Scalar(0, 0, 255));
		cv::putText(img, intToString(Segments.at(i).getxPos()) + " , " + intToString(Segments.at(i).getyPos()), 
						cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos() + 20), 1, 1, cv::Scalar(0, 255, 0));
		cv::putText(img, Segments.at(i).getType(), cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos() - 30), 1, 2, Segments.at(i).getColor());
		cv::putText(img, intToString(Segments.at(i).getarea()), cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos() - 20), 1, 1, cv::Scalar(0, 255, 0));
	}
}
