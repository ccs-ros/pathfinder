//Header containing custom lasses and functions for computer vision

#include <vector> 								//hue detection
#include <sstream> 								//hue detection
//#include <armadillo>							//linear algebra library
#include </usr/include/armadillo>
//#include <RcppArmadillo.h>
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

#define PI 3.14159265359

#define ARMA_USE_LAPACK
#define ARMA_USE_BLAS
#define ARMA_USE_ATLAS

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
		else if(name == "Red Test")
		{
			setHSVmin(cv::Scalar(20, 72, 161));
			setHSVmax(cv::Scalar(32, 126, 161));
			setYUVmin(cv::Scalar(38, 106, 159));
			setYUVmax(cv::Scalar(105, 121, 197));
			setColor(cv::Scalar(0, 0, 255));
		}
		else if(name == "Blue Test")
		{
			setHSVmin(cv::Scalar(93, 33, 113));
			setHSVmax(cv::Scalar(118, 202, 149));
			setYUVmin(cv::Scalar(43, 147, 88));
			setYUVmax(cv::Scalar(92, 164, 109));
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
			//setHSVmin(cv::Scalar(115, 134, 0));
			//setHSVmax(cv::Scalar(217, 255, 255));
			setHSVmin(cv::Scalar(0, 100, 94)); //paper red
			setHSVmax(cv::Scalar(202, 226, 134)); //paper red
			setYUVmin(cv::Scalar(15, 71, 140)); //46,110,137
			setYUVmax(cv::Scalar(163, 158, 255));//163,158,255
			setColor(cv::Scalar(0, 0, 255));		
		}
		else if(name == "Blue Side")
		{
			setHSVmin(cv::Scalar(93, 33, 113));
			setHSVmax(cv::Scalar(118, 202, 149));
			setYUVmin(cv::Scalar(185, 117, 0));
			setYUVmax(cv::Scalar(235, 166, 255));
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
			setHSVmin(cv::Scalar(136, 146, 48));
			setHSVmax(cv::Scalar(204, 244, 152));
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

	float getErr() {return error;}
	void setErr(float e) {error=e;}

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
	
	cv::Rect getCrop() {return boundingRectangle;}
	void setCrop(cv::Rect r) {boundingRectangle = r;}
	
	private:
	int xPos, yPos, area, areaObj;
	float error;
	string type;
	cv::Scalar HSVmin, HSVmax;
	cv::Scalar YUVmin, YUVmax;
	cv::Scalar Color;
	cv::Rect boundingRectangle;
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
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ERODE_SIZE, ERODE_SIZE));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DILATE_SIZE, DILATE_SIZE));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

//Converts given image to binary image of threshold values
cv::Mat filterHSV(const cv::Mat &frame, cv::Scalar colors_min, cv::Scalar colors_max, const int ERODE_SIZE, const int DILATE_SIZE, const int FILTER_SIZE, const int &SHOW_THRESH_IMAGE)
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

//Converts given image to binary image of threshold values
cv::Mat filterYUV(const cv::Mat &frame, cv::Scalar colors_min, cv::Scalar colors_max, const int ERODE_SIZE, const int DILATE_SIZE, const int FILTER_SIZE, const int &SHOW_THRESH_IMAGE)
{
	//Image variables
	cv::Mat img;

	//Process source image
	if(FILTER_SIZE>0) cv::GaussianBlur(frame, img, cv::Size(FILTER_SIZE,FILTER_SIZE), 0); //(9,9), (41,41), (11,11)
	else img = frame.clone();
	
	cv::cvtColor(img, img, cv::COLOR_BGR2YUV);

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
				Segment.setCrop(boundRect[i]);

				Segments.push_back(Segment);			
    		}
    	}
    }
	
    if(DRAW_HUE_TRACKING) drawHueDetection(frame, Segments, 1);
    
    return Segments;
}

//Function to find Lenght of sides of triangle
double DistanceTwoPoints(double x1, double y1, double x2, double y2)
{
    double x, y, distance;
    x = x2 - x1;
    y = y2 - y1;
    distance = x*x + y*y;
    distance = sqrt(distance);
    return distance;
}

//Calculate angle of triangle given three coordinates c++ code
void TriangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double a, b, c;
    double A, B, C;
    double total;

    int largestLength = 0;
    a = DistanceTwoPoints(x3, y3, x2, y2);
    b = DistanceTwoPoints(x1, y1, x3, y3);
    c = DistanceTwoPoints(x1, y1, x2, y2);
    
    A=acos((b*b+c*c-a*a)/(2*b*c));
    B=acos((a*a+c*c-b*b)/(2*a*c));
    C=acos((a*a+b*b-c*c)/(2*a*b));
    cout << "Angle = (" << A*(180/PI) << ", " << B*(180/PI) << ", " << C*(180/PI) << ")" << endl;
}

void filterBeaconResults(vector <TrackingObject> &red, vector <TrackingObject> &blue, int u_thresh)
{
	vector <int> idx, mem;
	vector <float> dists, threshs;
	float dist;
	int counter = 0;

	//find all blue objects that have a red object below that is vertically aligned
	for(int i=0; i<blue.size(); i++) 
		for(int j=0; j<red.size(); j++) 
			if(abs(blue[i].getxPos()-red[j].getxPos())<u_thresh && blue[i].getyPos()<red[j].getyPos())
			{
				float xtemp = blue[i].getxPos()-red[j].getxPos();
				float ytemp = blue[i].getyPos()-red[j].getyPos();
				threshs.push_back(sqrt(xtemp*xtemp+ytemp*ytemp));
				idx.push_back(i); //remember index
			}

	//find red object corrsponding to the blue object matching criteria
	for(int i=0; i<idx.size(); i++)
	{
		for(int j=0; j<red.size(); j++)
		{
			float xdiff = blue[idx[i]].getxPos()-red[j].getxPos();
			float ydiff = blue[idx[i]].getyPos()-red[j].getyPos();
			dist = sqrt(xdiff*xdiff+ydiff*ydiff);
			if(dist < threshs[i]) //threshold is relative
			{
				dists.push_back(dist);
				mem.push_back(j);
			}
		}
		if(dists.size()>2)
		{
			cout << "i = " << i << endl;
			cout << "Blue = (" << blue[idx[i]].getxPos() << ", " << blue[idx[i]].getyPos() << ")" << endl;

			for(int k=0; k<dists.size(); k++)
			{
				cout << "Red" << k << " = (" << red[mem[k]].getxPos() << ", " << red[mem[k]].getxPos() << ")" << endl;
			}
			counter++;
		}
		else
		{
			
		}
		
	}

	cout << counter << " solutions" << endl;
}

void simpleFindBeaconFromHues(vector <TrackingObject> red, vector <TrackingObject> blue)
{
	//Error for confidence calculation
	vector <float> u_err, blue_err, red_err;

	//Indices for blue candidates
	vector <int> bdx1, rdx1, bdx2, rdx2, bdx3, rdx3;
	float idx, jdx;

	//Temporary variables
	float xdiff, ydiff, vdiff, dist;

	//Find closest red object below blue object
	float distMin = 1e10;

	for(int i=0; i<blue.size(); i++)
	{	
		idx=-1;
		jdx=-1;
		for(int j=0; j<red.size(); j++)
		{
			xdiff = blue[i].getxPos()-red[j].getxPos();
			ydiff = blue[i].getyPos()-red[j].getyPos();
			dist = sqrt(xdiff*xdiff+ydiff*ydiff);

			if(blue[i].getyPos()<red[j].getyPos() && dist<distMin)
			{
				distMin=dist;
            		idx=i; //remember blue indices
            		jdx=j; //remember red indices
			}
		}

		if(idx != -1 && jdx != -1)
		{
			bdx1.push_back(idx); //remember blue indices
			rdx1.push_back(jdx); //remember red indices
		}
	}
	
	//Find closest red object above and left
	distMin = 1e10;
	for(int i=0; i<blue.size(); i++)
	{
		idx=-1;
		jdx=-1;
		for(int j=0; j<red.size(); j++)
		{
			xdiff = blue[i].getxPos()-red[j].getxPos();
			ydiff = blue[i].getyPos()-red[j].getyPos();
			dist = sqrt(xdiff*xdiff+ydiff*ydiff);

			if(blue[i].getxPos()>red[j].getxPos() && blue[i].getyPos()>red[j].getxPos() && dist<distMin)
			{
				distMin=dist;
            		idx=i; //remember blue indices
            		jdx=j; //remember red indices
			}
		}

		if(idx != -1 && jdx != -1)
		{
			bdx2.push_back(idx); //remember blue indices
			rdx2.push_back(jdx); //remember red indices
		}
	}

	//Find closest red object above and left
	distMin = 1e10;
	for(int i=0; i<blue.size(); i++)
	{
		idx=-1;
		jdx=-1;
		for(int j=0; j<red.size(); j++)
		{
			xdiff = blue[i].getxPos()-red[j].getxPos();
			ydiff = blue[i].getyPos()-red[j].getyPos();
			dist = sqrt(xdiff*xdiff+ydiff*ydiff);

			if(blue[i].getxPos()<red[j].getxPos() && blue[i].getyPos()>red[j].getxPos() && dist<distMin)
			{
				distMin=dist;
            		idx=i; //remember blue indices
            		jdx=j; //remember red indices
			}
		}

		if(idx != -1 && jdx != -1)
		{
			bdx3.push_back(idx); //remember blue indices
			rdx3.push_back(jdx); //remember red indices
		}
	}

	cout << "bdx1.size(),rdx1.size() = " << bdx1.size() << ", " << rdx1.size() << endl;
	cout << "bdx2.size(),rdx2.size() = " << bdx2.size() << ", " << rdx2.size() << endl;
	cout << "bdx3.size(),rdx3.size() = " << bdx3.size() << ", " << rdx3.size() << endl;

	//return Solutions;
}

vector <TrackingObject> findBeaconFromHues(vector <TrackingObject> red, vector <TrackingObject> blue)
{
	/*//For debugging
	TrackingObject reds, blues;
	vector <TrackingObject> red, blue;
	
	reds.setxPos(50);
	reds.setyPos(0);
	red.push_back(reds);
	reds.setxPos(0);
	reds.setyPos(519.6152);
	red.push_back(reds);
	reds.setxPos(-50);
	reds.setyPos(-10);
	red.push_back(reds);
	reds.setxPos(500);
	reds.setyPos(100);
	red.push_back(reds);

	blues.setxPos(0);
	blues.setyPos(173.20505);
	blue.push_back(blues);
	*/

	//Tolerance values
	int v_tol = 100;
	int h_tol = 100;
	float d_tol = 0.2;
	vector <float> d_tols;

	//Error for confidence calculation
	vector <float> blue_err, red_err, d_err, cand_err;

	//Indices for blue candidates
	vector <int> bdx, rdx;

	//Temporary variables
	float xdiff, ydiff, vdiff, dist;

	//Find candidates for blue objects
	for(int i=0; i<blue.size(); i++)
	{
		for(int j=0; j<red.size(); j++)
		{
			//If the object is algined horizontally and below
			if(abs(blue[i].getxPos()-red[j].getxPos())<h_tol && blue[i].getyPos()<red[j].getyPos())
			{
				//Calculate distance tolerance
            		xdiff = blue[i].getxPos()-red[j].getxPos();
				ydiff = blue[i].getyPos()-red[j].getyPos();
            		dist = sqrt(xdiff*xdiff+ydiff*ydiff);
            		d_tols.push_back(dist+3*dist*d_tol); //*4 due to non-equilateral triangle
            
            		//Candidate indices and error
            		blue_err.push_back(blue[i].getxPos()-red[j].getxPos());
            		bdx.push_back(i); //remember blue indices
            		rdx.push_back(j); //remember red indices
			}
		}
	}
	
	vector <float> x_cand, y_cand, sol_mat; //solution vectors
	vector <int> idx, mem; //indices for red candidates
	
	vector< vector<float> > xx, yy, ee;
	TrackingObject Sol;
	vector <TrackingObject> Sols;

	//Find candidates for red objects
	for(int i=0; i<bdx.size(); i++)
	{
		//Find indices from distance tolerance
		for(int j=0; j<red.size(); j++)
		{
			xdiff = blue[bdx[i]].getxPos()-red[j].getxPos();
			ydiff = blue[bdx[i]].getyPos()-red[j].getyPos();
			dist = sqrt(xdiff*xdiff+ydiff*ydiff);
			if(dist<d_tols[i]) 
			{
				mem.push_back(j);
				d_err.push_back(dist);
			}
		}
		
		//Find indices for vertical alignments (two points)
		for(int k=0; k<mem.size(); k++)
		{	
			for(int m=0; m<mem.size(); m++)
			{
				if(m!=k)
				{
					//cout << "mem[k] =" << mem[k] << endl;
					//cout << "mem[m] =" << mem[m] << endl;
					vdiff=abs(red[mem[k]].getyPos()-red[mem[m]].getyPos());
					if(vdiff<h_tol)
					{
						////Skip duplicate objects (TODO: improve remove duplicates)
						//for(int p=0; p<x_cand.size(); p++)
						//{
						//	if(x_cand[p] != red[mem[m]].getxPos() && y_cand[p] != red[mem[m]].getyPos())
						//	{
						//		x_cand.push_back(red[mem[m]].getxPos());
                    		//		y_cand.push_back(red[mem[m]].getyPos());
						//	}
						//}
						idx.push_back(mem[m]);
						red_err.push_back(d_err[m]+vdiff);
					}
				}
			}
		}
		
		//add first red object to index vector
		idx.push_back(rdx[i]);
		d_err.push_back(blue_err[i]);

		//Indices for all red candidates for blue i
		if(idx.size()>2)
		{
			for(int o=0; o<idx.size(); o++)
			{
				//vector of red coordinates for blue object i
				x_cand.push_back(red[idx[o]].getxPos());
				y_cand.push_back(red[idx[o]].getyPos());
				cand_err.push_back(blue_err[o]);
			}
			//vector of coordinate vectors for all possible solutions
			xx.push_back(x_cand);
			yy.push_back(y_cand);
			ee.push_back(cand_err);
		}

		//Clear temp vectors
		idx.clear();
		mem.clear();
		x_cand.clear();
		y_cand.clear();
		cand_err.clear();
	}
	
	cout << "xx.size() = " << xx.size() << endl;

	//Find best guess
	TrackingObject Solution;
	vector <TrackingObject> Solutions;
	float err_sum, err_sump;
	int smallestError = 1e8;
	int idx_err;
	if(xx.size()>1)
	{
		for(int i=0; i<xx.size(); i++)
		{	
			x_cand=xx[i];
			y_cand=yy[i];
			cand_err=ee[i];
			err_sum = 0;
			for(int j=0; j<x_cand.size(); j++)
			{
				err_sum += cand_err[j];
				//cout << "cand_err[" << j << "] =" << cand_err[j] << endl;
			}
			cout << "err sum = " << err_sum << endl;

			if(err_sum<smallestError)
			{
				idx_err=i;
				smallestError=err_sum;
			}
		}

		x_cand=xx[idx_err];
		y_cand=yy[idx_err];
		cand_err=ee[idx_err];
		for(int i=0; i<x_cand.size(); i++)
		{
			cout << "cand_err[" << i << "] =" << cand_err[i] << endl;
			Solution.setxPos(x_cand[i]);
			Solution.setyPos(y_cand[i]);
			Solutions.push_back(Solution);
		}
	}
	else if(xx.size()==1) //If one solution found okay
	{
		x_cand=xx[0];
		y_cand=yy[0];
		cand_err=ee[0];
		for(int i=0; i<x_cand.size(); i++)
		{
			cout << "cand_err[" << i << "] =" << cand_err[i] << endl;
			Solution.setxPos(x_cand[i]);
			Solution.setyPos(y_cand[i]);
			Solutions.push_back(Solution);
		}
	}
	else
	{
	}

	cout << "x_cand.size() = " << x_cand.size() << endl;
	cout << "y_cand.size() = " << y_cand.size() << endl;

	return Solutions;
}

float findHomingBearing(vector <TrackingObject> Objects, float fov_angle, float focal_length, int width, int height)
{
	/* Created on 4/28/14
	 * Calculates the bearing based on the homing beacon vertices
	 */

	//Store coordinates into vector
	arma::vec u, v;
	u << Objects[0].getxPos() << Objects[1].getxPos() << Objects[2].getxPos();
	v << Objects[0].getyPos() << Objects[1].getyPos() << Objects[2].getyPos();

	//Calculate scaling factor
	fov_angle=fov_angle*PI/180;
	float hyp = focal_length*sin(fov_angle)/sin(PI-(fov_angle+fov_angle/2));
	float beta = atan2(height/2,width/2);
	float w2 = sin(beta)*hyp;
	float s = w2/(width/2); //dist/pixels
	cout << "hyp,beta,we,s =" <<  hyp << "," << beta << "," << w2 << "," << s << endl;
	//Organize triangle vertices
	double u1, u2, u3, v1, v2, v3;
	arma::uvec idx1, idx2, idx3;
	idx1 = arma::find(u==u.min());
	u1 = (u(idx1(0))-width/2)*s/focal_length;
	v1 = (v(idx1(0))-height/2)*s/focal_length;
	idx3 = arma::find(v==v.max());
	u3 = (u(idx3(0))-width/2)*s/focal_length;
	v3 = (v(idx3(0))-height/2)*s/focal_length;
	idx2 = arma::find(u==u.max());
	u2 = (u(idx2(0))-width/2)*s/focal_length;
	v2 = (v(idx2(0))-height/2)*s/focal_length;

	cout << "u=" << u1 << "," << u2 << "," << u3 << endl;
/*
	//Finds ratio
	float midU, midV, s, h, r, R, bearing;
	midU = (u3+u1)/2;
	midV = (v3+v1)/2;
	R = 2/(tan(PI/3));
	h = sqrt((midX-u2)*(midX-u2)+(midY-v2)*(midY-v2));
	s = sqrt((u3-u1)*(u3-u1)+(v3-v1)*(v3-v1));
	r = s/h;
	if (r/R<1) bearing = acos(r/R);
	else bearing = 0; 
*/

/*
	//Find interior angles
	TriangleAngleCalculation(u1, v1, u2, v2, u3, v3);
*/

//If algorithm diverges
int error;

double xa, xb, xc, ya, yb, yc, S1, S2, S3;

xa = -0.85725;
xb = 0.85725;
xc = 0;
ya = 0.52075;
yb = 0.52075;
yc = -0.52075;
S1 = sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb));
S2 = sqrt((xb-xc)*(xb-xc)+(yb-yc)*(yb-yc));
S3 = sqrt((xc-xa)*(xc-xa)+(yc-ya)*(yc-ya));

double ua_f, ub_f, uc_f;
double va_f, vb_f, vc_f;

ua_f = u1;
ub_f = u2;
uc_f = u3;


va_f = v1;
vb_f = v2;
vc_f = v3;

//for debugging
//ua_f = -103.98;
//ub_f = 103.98;
//uc_f = 0;
//
//va_f = 103.98*1/3*sqrt(3);
//vb_f = 103.98*1/3*sqrt(3);
//vc_f = -103.98*2/3*sqrt(3);

arma::vec na, nb, nc;
na << ua_f << va_f << 1;
na = na/arma::norm(na,2);
nb << ub_f << vb_f << 1;
nb = nb/arma::norm(nb,2);
nc << uc_f << vc_f << 1;
nc = nc/arma::norm(nc,2);

double cBeta1, cBeta2, cBeta3;
cBeta1 = na(0)*nb(0)+na(1)*nb(1)+na(2)*nb(2);
cBeta2 = nb(0)*nc(0)+nb(1)*nc(1)+nb(2)*nc(2);
cBeta3 = nc(0)*na(0)+nc(1)*na(1)+nc(2)*na(2);
//cout << "cB =" << cBeta1 << endl;

arma::vec X, X0, FX;
X << 0 << 5;
X0 << 0 << 10;
FX << 10 << 10 << 10;

short int count = 0;

double x, z, dxa, dxb, dxc, a, b, c, eq1, eq2, eq3;
arma::mat J(3,2);

while (arma::norm(FX)>1e-8 && count<25 && norm(X0-X)>1e-5)
{
    count = count+1;
    X0 = X;
    x = X(0);
    z = X(1);
    
    dxa = x-xa;
    dxb = x-xb;
    dxc = x-xc;
    
    a = dxa*dxa+ya*ya+z*z;
    b = dxb*dxb+yb*yb+z*z;
    c = dxc*dxc+yc*yc+z*z;
    
    eq1 = (a)+(b)-2*sqrt(a)*sqrt(b)*cBeta1-S1*S1;
    eq2 = (b)+(c)-2*sqrt(b)*sqrt(c)*cBeta2-S2*S2;
    eq3 = (c)+(a)-2*sqrt(c)*sqrt(a)*cBeta3-S3*S3;
    
    FX << eq1 << eq2 << eq3;
    
    J << 4*x-2*xa-2*xb-(cBeta1*(2*x-2*xa)*sqrt(b))/sqrt(a)-(cBeta1*(2*x-2*xb)*sqrt(a))/sqrt(b) << 4*z-(2*z*cBeta1*sqrt(b))/sqrt(a)-(2*z*cBeta1*sqrt(a))/sqrt(b) << arma::endr
      << 4*x-2*xb-2*xc-(cBeta2*(2*x-2*xb)*sqrt(c))/sqrt(b)-(cBeta2*(2*x-2*xc)*sqrt(b))/sqrt(c) << 4*z-(2*z*cBeta2*sqrt(c))/sqrt(b)-(2*z*cBeta2*sqrt(b))/sqrt(c) << arma::endr
      << 4*x-2*xa-2*xc-(cBeta3*(2*x-2*xa)*sqrt(c))/sqrt(a)-(cBeta3*(2*x-2*xc)*sqrt(a))/sqrt(c) << 4*z-(2*z*cBeta3*sqrt(c))/sqrt(a)-(2*z*cBeta3*sqrt(a))/sqrt(c) << arma::endr;
    
	//X.print("X ");
	//J.print("J ");
	//FX.print("FX ");
	//cout << arma::norm(FX) << endl;

	if(arma::norm(FX)>1e5) {error=1; break;}
	else error=0;

    X = X-arma::inv(J.st()*J)*J.st()*FX;
	
	//arma::mat JJ = arma::inv(J.st()*J)*J.st();
	//JJ.print("JJ = ");
}

//cout << count << endl;
//cout << arma::norm(FX) << endl;
x = X(0);
z = X(1);

double d, gamma;
d = sqrt(x*x+z*z);
gamma = atan2(x,z);
cout << d << endl;

	return gamma;
}
