//Header containing custom lasses and functions for computer vision

#include <vector> 								//hue detection
#include <sstream> 								//hue detection
#include <armadillo>							//linear algebra library
#include <image_transport/image_transport.h> 	//publush and subscribe to ros images
#include <cv_bridge/cv_bridge.h> 				//convert between ros and opencv image formats
#include <sensor_msgs/image_encodings.h> 		//constants for image encoding
#include <opencv2/imgproc/imgproc.hpp> 			//image processing
#include <opencv2/highgui/highgui.hpp> 			//gui handling
#include <opencv2/core/core.hpp>				//core functions and data types
#include <opencv2/features2d/features2d.hpp>	//2d features framework
#include <opencv2/nonfree/features2d.hpp>		//SURF algorithms
#include <opencv2/calib3d/calib3d.hpp>			//camera calibration and 3d reconstruction
#include <opencv2/legacy/legacy.hpp>			//brute force matching

using namespace std;

class TrackingObject
{
	public:
	TrackingObject(void){};
	~TrackingObject(void){};

	TrackingObject(std::string name)
	{
		setType(name);
		
		if(name == "SideOne") //blue
		{
			setHSVmin(cv::Scalar(104, 39, 73));
			setHSVmax(cv::Scalar(119, 118, 190));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(255, 0, 0));
		}
		else if(name == "SideTwo") //gold
		{
			setHSVmin(cv::Scalar(20, 72, 161));
			setHSVmax(cv::Scalar(32, 126, 161));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));
		}
		else if(name == "Pre-Cached")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Cropped")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Segment")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Red Ball")
		{
			setHSVmin(cv::Scalar(115, 134, 0));
			setHSVmax(cv::Scalar(217, 255, 255));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Blue Sample")
		{
			setHSVmin(cv::Scalar(104, 39, 73));
			setHSVmax(cv::Scalar(119, 118, 190));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Pink Sample")
		{
			setHSVmin(cv::Scalar(167, 95, 153));
			setHSVmax(cv::Scalar(196, 215, 222));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Red Sample")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Orange Sample")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Wood Sample")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Brass Sample")
		{
			setHSVmin(cv::Scalar(0, 0, 0));
			setHSVmax(cv::Scalar(0, 0, 0));
			setYUVmin(cv::Scalar(0, 0, 0));
			setYUVmax(cv::Scalar(0, 0, 0));
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
	
	cv::Scalar getYUVmin() {return YUVmin;}
	cv::Scalar getYUVmax() {return YUVmax;}

	void setHSVmin(cv::Scalar min) {HSVmin=min;}
	void setHSVmax(cv::Scalar max) {HSVmax=max;}
	
	void setYUVmin(cv::Scalar min) {YUVmin=min;}
	void setYUVmax(cv::Scalar max) {YUVmax=max;}

	std::string getType() {return type;}
	void setType(std::string t) {type = t;}

	cv::Scalar getColor() {return Color;}
	void setColor(cv::Scalar c) {Color = c;}
	
	vector<cv::Rect> getCrop() {return boundingRectangle;}
	void setCrop(vector<cv::Rect> r) {boundingRectangle = r;}
	
	private:
	int xPos, yPos, area, areaObj;
	string type;
	cv::Scalar HSVmin, HSVmax;
	cv::Scalar YUVmin, YUVmax;
	cv::Scalar Color;
	vector<cv::Rect> boundingRectangle;
};

//Returns number of circles in frame
int findHoughCircles(const cv::Mat &frame, const int &DRAW_CIRCLES)
{
	//Image variables
    cv::Mat src, gray;
    
    //Hough Transform variables
    vector<cv::Vec3f> circles;
    
    //Copy input image
    src = frame;
    
    //Process source image
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);		//Convert image to grayscale
    //cv::GaussianBlur(gray, gray, cv::Size(11,11), 2, 2);	//Smooth grayscale image
    cv::medianBlur(gray, gray, 7);
    
    //Hough Transform to find circles
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/8, 300, 5, 1, 1000);
    
    //Find detected circles
	for(size_t i = 0; i < circles.size(); i++ )
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));	//find center of circle
		int radius = cvRound(circles[i][2]);								//find radius of circle
		
		if(DRAW_CIRCLES) circle(src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );		//draw center    
		if(DRAW_CIRCLES) circle(src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );	//draw outline
		//cout << "center : " << center << "\nradius : " << radius << endl;
	}
    
    if(DRAW_CIRCLES) {imshow("Hough Circle Transform (findHoughCircles)", src);}
    
    return circles.size();
}

string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//Draws detected hues from TrackingObject class in given frame
void drawHueDetection(const cv::Mat &frame, vector <TrackingObject> &Segments, const int &DRAW_HUE_TRACKING)
{
	cv::Mat img = frame.clone();
	
	for (int i = 0; i < Segments.size(); i++)
	{
		Segments.at(i).getxPos();
		Segments.at(i).getyPos();
		Segments.at(i).getarea();

		cv::circle(img, cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos()), 10, cv::Scalar(0, 0, 255));
		cv::putText(img, intToString(Segments.at(i).getxPos()) + " , " + intToString(Segments.at(i).getyPos()), cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos() + 20), 1, 1, cv::Scalar(0, 255, 0));
		cv::putText(img, Segments.at(i).getType(), cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos() - 30), 1, 2, Segments.at(i).getColor());
		cv::putText(img, intToString(Segments.at(i).getarea()), cv::Point(Segments.at(i).getxPos(), Segments.at(i).getyPos() - 20), 1, 1, cv::Scalar(0, 255, 0));
	}
	
	cv::imshow("Hue Tracking (findCandidates)", img);
}

void morphImg(cv::Mat &thresh, const int ERODE_SIZE, const int DILATE_SIZE)
{
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(ERODE_SIZE, ERODE_SIZE));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(DILATE_SIZE, DILATE_SIZE));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

//Converts given image to binary image of threshold values
cv::Mat filterColors(const cv::Mat &frame, cv::Scalar colors_min, cv::Scalar colors_max, const int ERODE_SIZE, const int DILATE_SIZE, const int FILTER_SIZE, const int &SHOW_THRESH_IMAGE)
{
	//Image variables
	cv::Mat img;

	//Process source image
	if(FILTER_SIZE>0) cv::GaussianBlur(frame, img, cv::Size(FILTER_SIZE,FILTER_SIZE), 0); //(9,9), (41,41), (11,11)
	else img = frame.clone();
	
	cv::cvtColor(img, img, cv::COLOR_BGR2HSV);

	cv::inRange(img, colors_min, colors_max, img); 

	morphImg(img, ERODE_SIZE, DILATE_SIZE);			

	if(SHOW_THRESH_IMAGE) cv::namedWindow("Threshold Image (filterColors)");
	if(SHOW_THRESH_IMAGE) cv::imshow("Threshold Image (filterColors)", img);
	
	return img;
}

cv::Mat filterWhite(const cv::Mat &frame, const int THRESH_MIN, const int THRESH_MAX, const int THRESH_TYPE, const int ERODE_SIZE, const int DILATE_SIZE, const int &SHOW_THRESH_IMAGE)
{
	//Split frame into BGR channels 
	cv::Mat R, G, B;
	cv::Mat channel[3];
	split(frame, channel);
	
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
	threshold(B, ch1, THRESH_MIN, THRESH_MAX, THRESH_TYPE);
	threshold(G, ch2, THRESH_MIN, THRESH_MAX, THRESH_TYPE);
	threshold(R, ch3, THRESH_MIN, THRESH_MAX, THRESH_TYPE);
		
	//AND together B, G, R channels
	cv::bitwise_and(ch1, ch2, temp);
	cv::bitwise_and(ch3, temp, res);

	//Display resulting image
	morphImg(res, ERODE_SIZE, DILATE_SIZE);
	
	return res;
}

// Returns vector of class TrackingObject of segments constaining circles
vector <TrackingObject> findCandidates(const cv::Mat &frame, const cv::Mat imgThresh, string type, const int &MAX_NUM_OBJECTS, const int &MIN_OBJECT_AREA, const int &MAX_OBJECT_AREA, const int &DRAW_HUE_TRACKING)
{
	//Object variables
	vector <TrackingObject> Segments;	//vector of candidates
	TrackingObject Segment(type);	//candidate of interest
	
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
				//Find bounding box for object
				cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
				boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

				//cv::rectangle(img, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255,0,255), 2, 8, 0 );
				//cv::Mat croppedImage = img(boundRect[i]).clone();

				Segment.setxPos(moment.m10/moment.m00);
				Segment.setyPos(moment.m01/moment.m00);
				Segment.setarea(moment.m00);
				Segment.setType(Segment.getType());
				Segment.setColor(Segment.getColor());
				Segment.setCrop(boundRect);

				Segments.push_back(Segment);			
    		}
    	}
    }
	
    if(DRAW_HUE_TRACKING) drawHueDetection(frame, Segments, 1);
    
    return Segments;
}
