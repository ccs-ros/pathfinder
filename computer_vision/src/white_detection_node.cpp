//Includes all the headers necessary to use the most common public pieces of the ROS system.
//#include <iostream>
//#include <stdio.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
image_transport::Publisher pub; //Use method of ImageTransport to create image publisher
ros::Publisher pub2;

static const char OUTPUT_DATA_TOPIC[] = "det/mis_out_data";

const bool DRAW_HUE_TRACKING = true;		//findCandidates function 

//Default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;

//White detection constants
const double threshold_val = 220;
const double max_BINARY_value = 255;
const int threshold_type = 3;

//Max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;

//Minimum and maximum object hue area
const int MIN_OBJECT_AREA = 4 * 4;
const int MAX_OBJECT_AREA = 85 * 85;


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


void morphImg(cv::Mat &thresh);
vector <TrackingObject> findCandidates(const cv::Mat &frame, const cv::Mat imgThresh);
void drawHueDetection(const cv::Mat &frame, vector <TrackingObject> &Segments);
string intToString(int number);

//Help function
static void help()
{
	std::cout << "This program detects white objects."
		<< std::endl << "" << std::endl << std::endl;
}

int x_object = 0;
int y_object = 0;
int object_seen = 0;
int object_area = 0;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//start time to record frequency
	double begin = ros::Time::now().toSec();
	
	//Output structure	
	computer_vision::Beacon outData;
	
	//Image variables
	vector <TrackingObject> Segments;

	//Convert from the ROS image message to a CvImage
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
        	//Always copy, returning a mutable CvImage; OpenCV expects color images to use BGR channel order.
        	cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
        	//if there is an error during conversion, display it
        	ROS_ERROR("white_detection::white_detection_node.cpp::cv_bridge exception: %s", e.what());
        	return;
	}
	
	//Split frame into BGR channels 
	cv::Mat R, G, B;
	cv::Mat channel[3];
	split(cv_ptr->image, channel);
	
	//Convert channels to 8 bit unsigned format
	channel[0].convertTo(B, CV_8U);
	channel[1].convertTo(G, CV_8U);
	channel[2].convertTo(R, CV_8U);
	
	//Empty matrices for binary images
	cv::Mat ch1 = cv::Mat(B.size(), CV_8UC1);
	cv::Mat ch2 = cv::Mat(G.size(), CV_8UC1);
	cv::Mat ch3 = cv::Mat(R.size(), CV_8UC1);
	cv::Mat temp = cv::Mat(R.size(), CV_8UC1);
	cv::Mat res = cv::Mat(R.size(), CV_8UC1);

	//Convert channels to binary images
	threshold(B, ch1, threshold_val, max_BINARY_value, threshold_type);
	threshold(G, ch2, threshold_val, max_BINARY_value, threshold_type);
	threshold(R, ch3, threshold_val, max_BINARY_value, threshold_type);
		
	//AND together B, G, R channels
	cv::bitwise_and(ch1, ch2, temp);
	cv::bitwise_and(ch3, temp, res);

	//Display resulting image
	morphImg(res);
	imshow("Resulting Image", res);	
	cv::waitKey(1);
  	
	Segments = findCandidates(cv_ptr->image, res);
	    
	if(Segments.size()>0)
	{	
		object_seen=1;
		x_object=Segments[0].getxPos();
		y_object=Segments[0].getyPos();
		object_area=Segments[0].getarea();
	} else object_seen=0;

	outData.centerx = x_object;
	outData.centery = y_object;
	outData.areaobject = object_area;
	outData.object = object_seen;
    
	pub2.publish(outData);
	//pub.publish(cv_ptr->toImageMsg());
	
	//end timer and print frequency
	double end = ros::Time::now().toSec();
	ROS_INFO("Sample Rate = %f", 1/(end-begin));
        
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
	
    cv::namedWindow("Resulting Image", CV_WINDOW_AUTOSIZE);  

    image_transport::Subscriber sub = it.subscribe("logitech_c920/image_raw", 1, imageCallback);
    //image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    
    cv::destroyWindow("Resulting Image"); //OpenCV HighGUI call to destroy window on shut-down.
 
    pub2 = nh.advertise<computer_vision::Beacon>(OUTPUT_DATA_TOPIC, 1);
	
    ros::Rate r(30);
    ros::spin();

    ROS_INFO("white_detection::white_detection_node.cpp::No error.");
 
}

//Erodes and dilates image for tracking
void morphImg(cv::Mat &thresh)
{
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(12, 12));

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

string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}
