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

#define PI 3.14159265359

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
			//setHSVmin(cv::Scalar(115, 134, 0));
			//setHSVmax(cv::Scalar(217, 255, 255));
			setHSVmin(cv::Scalar(0, 100, 94)); //paper red
			setHSVmax(cv::Scalar(202, 226, 134)); //paper red
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

float findHomingBearing(vector <TrackingObject> Objects, float focal_length)
{
	/* Created on 4/28/14
	 * Calculates the bearing based on the homing beacon vertices
	 */

	//Store coordinates into vector
	arma::vec u, v;
	u << Objects[0].getxPos() << Objects[1].getxPos() << Objects[2].getxPos();
	v << Objects[0].getyPos() << Objects[1].getyPos() << Objects[2].getyPos();

	//Organize triangle vertices
	float u1, u2, u3, v1, v2, v3;
	arma::uvec idx1, idx2, idx3;
	idx1 = arma::find(u==u.min());
	u1 = (u(idx1(0))-320)/focal_length;
	v1 = (v(idx1(0))-240)/focal_length;
	idx3 = arma::find(v==v.max());
	u3 = (u(idx3(0))-320)/focal_length;
	v3 = (v(idx3(0))-240)/focal_length;
	idx2 = arma::find(u==u.max());
	u2 = (u(idx2(0))-320)/focal_length;
	v2 = (v(idx2(0))-240)/focal_length;

	//cout << "u=" << u1 << "," << u2 << "," << u3 << endl;
/*
	//Find ratio
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
float xa, xb, xc, ya, yb, yc, S1, S2, S3;
xa = -103.98;
xb = 103.98;
xc = 0;
ya = 103.98/3*sqrt(3);
yb = 103.98/3*sqrt(3);
yc = -(2*103.98)/3*sqrt(3);
S1 = sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb));
S2 = sqrt((xb-xc)*(xb-xc)+(yb-yc)*(yb-yc));
S3 = sqrt((xc-xa)*(xc-xa)+(yc-ya)*(yc-ya));

float ua_f, ub_f, uc_f;
float va_f, vb_f, vc_f;
/*
ua_f = u1;
ub_f = u2;
uc_f = u3;


va_f = v1;
vb_f = v2;
vc_f = v3;
*/
ua_f = -103.98;
ub_f = 103.98;
uc_f = 0;

va_f = 103.98*1/3*sqrt(3);
vb_f = 103.98*1/3*sqrt(3);
vc_f = -103.98*2/3*sqrt(3);

arma::vec na, nb, nc;
na << ua_f << va_f << 1;
na = na/arma::norm(na);
nb << ub_f << vb_f << 1;
nb = nb/arma::norm(nb);
nc << uc_f << vc_f << 1;
nc = nc/arma::norm(nc);

float cBeta1, cBeta2, cBeta3;
cBeta1 = na(0)*nb(0)+na(1)*nb(1)+na(2)*nb(2);
cBeta2 = nb(0)*nc(0)+nb(1)*nc(1)+nb(2)*nc(2);
cBeta3 = nc(0)*na(0)+nc(1)*na(1)+nc(2)*na(2);
cout << "cB =" << cBeta1 << endl;

arma::vec X, X0, FX;
X << 0 << 5;
X0 << 0 << 10;
FX << 10 << 10 << 10;

short int count = 0;

float x, z, dxa, dxb, dxc, a, b, c, eq1, eq2, eq3;
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
    
    X = X-arma::inv(J.st()*J)*J.st()*FX;
}

cout << count << endl;
cout << arma::norm(FX) << endl;
x = X(0);
z = X(1);

float d, gamma;
d = sqrt(x*x+z*z);
gamma = atan2(x,z);
cout << d << endl;

	return gamma;
}
