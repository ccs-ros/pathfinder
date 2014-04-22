/**
 * det_homing_node.cpp version 3.4
 * 
 * Author: Jared Strader
 * Last Update: 4/1/14 
 *
 * This node subscribes to the logitech_c920/image_raw topic then finds the homography 
 * matrix of the homing beacon using the training image using orb, brute force matching, 
 * and the ratio-test.
 *
 * CHANGES SINCE V2:	1. trackHomingBeacon function now return homography matrix
 *	   	     			2. surf algorithms change to orb algorithms
 *	    	        	3. FLANN matcher changed to brute force matcher
 *		     			4. matching verification changed from distance check to ratio-test
 *	 	SINCE V3.0:
 *		     			5. code partially re-organized and unused code removed
 * 	 	SINCE V3.1:
 *		   				6. image display for tracked beacon moved into seperate function
 *		    			7. publishing estimate structure in place
 *   	SINCE V3.2:
 *						8. added function for homography decomposition (not working yet)
 *    	SINCE V3.3:
 *						9. new method for homography decomposition (fairly stable)
 *						10.confidence added for homing beacon tracking (in progress)
 *
 * NOTES: 
 * 
 * 1. When running the node if the error "error while loading shared libraries: libarmadillo.so.4: cannot
 * open shared object file: No such file or directory" try running sudo ldconfig and catkin_make clean
 * 
 * 2. Armadillo must be installed from source. If installed using apt-get install, remove and 
 * reinstall the updated version of armadillo c++		
 */
 
#include <ros/ros.h> 								//includes headers common for ros system
#include <vector>									//standard c++ vectors
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
#include </usr/include/armadillo>					//linear algebra libraries
#include <math.h>									//standard c++ math libaries

using namespace std;
namespace enc = sensor_msgs::image_encodings;
 
//Settings
const bool DRAW_FOUND_MATCHES = true;
const bool SHOW_ORIGIONAL_IMAGE = false;
const bool HZ = false; //print tracking frequency
const bool PRINT_POSE = false;
const bool DEBUG_HOMOGRAPHY = false;

//Definitions
static const string home = getenv("HOME");
static const char IMAGE_TOPIC[] = "logitech_c920/image_raw";
static const char OUTPUT_DATA_TOPIC[] = "det/mis_out_data";
static const string BEACON_IMG_ADDRESS = home + "/catkin_ws/src/computer_vision/images/pcb.png";//idcard2.jpg";
static const string BEACON_FILE_ADDRESS = home + "/catkin_ws/src/computer_vision/images/objectData.yml";
static const char WINDOW[] = "Display (det_homing_node)";

//Function headers
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image);
cv::Mat trackHomingBeacon(const cv::Mat &frame);
void findCenter(const cv::Mat &imgA, const vector<cv::KeyPoint> &keypointsA, const cv::Mat &imgB, const vector<cv::KeyPoint> &keypointsB, const vector<cv::DMatch> good_matches, const cv::Mat H);
void findRotation(cv::Mat &homography);
template <typename T> int sgn(T val);

//Global declarations
ros::Publisher pub;
int confidence = 0;
int cx = 0, cy = 0;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "det_homing_node");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
    
   cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
   cv::destroyWindow(WINDOW);

   image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
   pub = nh.advertise<computer_vision::Beacon>(OUTPUT_DATA_TOPIC, 1); 
  
   ROS_INFO("Running det_homing_node...");
   ros::spin();
   
   return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    double begin = ros::Time::now().toSec(); //start timer
    
    //structures for subscribing and publishing
    cv_bridge::CvImagePtr cv_ptr;
    computer_vision::Beacon pose_estimate;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("computer_vision::det_homing_node.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    
    if(SHOW_ORIGIONAL_IMAGE==true) 
    {
		cv::imshow(WINDOW, cv_ptr->image);
		cv::waitKey(1);
    }

    cv::Mat H = trackHomingBeacon(cv_ptr->image); //track homing beacon and return homography
	//cout << "H = " << endl << H << endl;
	//cout << "confidence = " << confidence << endl;
	/*arma::mat tempH(3,3);
	tempH(0,0) = H.at<double>(0,0);
	tempH(0,1) = H.at<double>(0,1);
	tempH(0,2) = H.at<double>(0,2);
	tempH(1,0) = H.at<double>(1,0);
	tempH(1,1) = H.at<double>(1,1);
	tempH(1,2) = H.at<double>(1,2);
	tempH(2,0) = H.at<double>(2,0);
	tempH(2,1) = H.at<double>(2,1);
	tempH(2,2) = H.at<double>(2,2);*/
    //if(arma::max(arma::max(tempH))>0.00001) 
    //findRotation(H);
    
    //estimation structure
    pose_estimate.x=0; pose_estimate.y=0; pose_estimate.z=0;
    pose_estimate.pitch=90; pose_estimate.roll=90; pose_estimate.yaw=90; pose_estimate.heading=45;
    pose_estimate.centerx=cx; pose_estimate.centery=cy;
    
    if(confidence > 50) pose_estimate.object=1;
    else pose_estimate.object=0;
    
    if(confidence<100 && sqrt(cx*cx+cy*cy)-sqrt(cx*cx+cy*cy)<20) confidence = confidence + 20;
	else confidence = confidence - 25;
	
	if(confidence > 0) confidence = confidence - 20;
    
    pub.publish(pose_estimate); //publish data
    
    double end = ros::Time::now().toSec(); //stop timer
    if(HZ) ROS_INFO("Sample Rate = %f", 1/(end-begin)); //print execution time
}

cv::Mat trackHomingBeacon(const cv::Mat &imgB)
{
    cv::Mat imgA = cv::imread(BEACON_IMG_ADDRESS, CV_LOAD_IMAGE_GRAYSCALE); 
    cv::Mat H;
 
    if (!imgA.data) ROS_WARN("Cannot load beacon image!");
    
    //feature variables
    vector<cv::KeyPoint> keypointsA, keypointsB;
    cv::Mat descriptorsA, descriptorsB;
    vector<vector<cv::DMatch> > matches; 

    //detector, extractor, and matcher declarations
    cv::ORB detector;
    cv::ORB extractor;
    cv::BFMatcher matcher;

    //computer keypoints
    double t = (double)cv::getTickCount();
    detector.detect(imgA, keypointsA);
    detector.detect(imgB, keypointsB);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if(HZ) ROS_INFO("detection time [s]: %f", t/1.0);
    
    //extract descriptors
    t = (double)cv::getTickCount();
    extractor.compute(imgA, keypointsA, descriptorsA);
    extractor.compute(imgB, keypointsB, descriptorsB);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if(HZ) ROS_INFO("extraction time [s]: %f", t);
    
	//match descriptors
    t = (double)cv::getTickCount();
    matcher.knnMatch(descriptorsA, descriptorsB, matches, 2);  // Find two nearest matches 
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if(HZ) ROS_INFO("matching time [s]: %f", t);
    
    //Find good matches using the ratio test
    vector<cv::DMatch> good_matches;

    for (int i = 0; i < matches.size(); ++i)
    {
		const float ratio = 0.8; // Value from Lowe's paper
		if (matches[i][0].distance < ratio * matches[i][1].distance)
		{
            good_matches.push_back(matches[i][0]);
		}
    }
	//cout << "Good Matches = " << good_matches.size() << endl;
    if (good_matches.size() >= 4 && good_matches.size() <= 85) //[8,30]
    {
    	
		vector<cv::Point2f> obj;
		vector<cv::Point2f> scene;

		for (unsigned int i = 0; i < good_matches.size(); i++)
		{
            //-- Get the keypoints from the good matches
	 	   obj.push_back(keypointsA[good_matches[i].queryIdx].pt);
		   scene.push_back(keypointsB[good_matches[i].trainIdx].pt);
		}

		H = findHomography(obj, scene, CV_RANSAC);
		
		//Find center of tracked image
		findCenter(imgA, keypointsA, imgB, keypointsB, good_matches, H);
		
		return H;
    }
	
	H = cv::Mat::zeros(3, 3, CV_32F);
    //H = findHomography(obj, scene, CV_RANSAC);
    return H; 
}

void findCenter(const cv::Mat &imgA, const vector<cv::KeyPoint> &keypointsA, const cv::Mat &imgB, const vector<cv::KeyPoint> &keypointsB, const vector<cv::DMatch> good_matches, const cv::Mat H)
{      
	//Get the corners from the training image
	std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0); 
	obj_corners[1] = cvPoint(imgA.cols, 0);
	obj_corners[2] = cvPoint(imgA.cols, imgA.rows); 
	obj_corners[3] = cvPoint(0, imgA.rows);
	std::vector<cv::Point2f> scene_corners(4);

	perspectiveTransform(obj_corners, scene_corners, H);

	//Draw lines between the corners of beacon
	if(DRAW_FOUND_MATCHES==true) 
	{
		cv::Mat imgMatch;
		drawMatches(imgA, keypointsA, imgB, keypointsB, good_matches, imgMatch);
		cv::line(imgMatch, scene_corners[0] + cv::Point2f(imgA.cols, 0), scene_corners[1] + cv::Point2f(imgA.cols, 0), cv::Scalar(255, 255, 0), 2); //top of beacon
		cv::line(imgMatch, scene_corners[1] + cv::Point2f(imgA.cols, 0), scene_corners[2] + cv::Point2f(imgA.cols, 0), cv::Scalar(255, 0, 0), 2);
		cv::line(imgMatch, scene_corners[2] + cv::Point2f(imgA.cols, 0), scene_corners[3] + cv::Point2f(imgA.cols, 0), cv::Scalar(255, 0, 0), 2);
		cv::line(imgMatch, scene_corners[3] + cv::Point2f(imgA.cols, 0), scene_corners[0] + cv::Point2f(imgA.cols, 0), cv::Scalar(255, 0, 0), 2);
		cv::imshow(WINDOW, imgMatch);
		cv::waitKey(1);
	}
	
	//Find center of beacon (needs to be moved out of drawing function
	cv::Point2f p1 = scene_corners[0] + cv::Point2f(imgA.cols, 0);
	cv::Point2f p2 = scene_corners[1] + cv::Point2f(imgA.cols, 0);
	cv::Point2f p3 = scene_corners[2] + cv::Point2f(imgA.cols, 0);
	cv::Point2f p4 = scene_corners[3] + cv::Point2f(imgA.cols, 0);
	cv::Point2f center = (p3*0.5 + p1*0.5);
	
	cx = center.x; 
	cy = center.y;
}

void findRotation(cv::Mat &homography)
{
	//variable declarations
	arma::mat H(3,3), M(3,3), temp(2,2), R(3,3), R2(3,3);
	arma::mat I(3, 3, arma::fill::eye);
	arma::vec na1(3), na2(3), na3(3);
	arma::vec nb1(3), nb2(3), nb3(3);
	arma::vec ta(3), tb(3), es(3), v(3), u(3);
	
	u << 0 << 1 << 0 << arma::endr;
	v << 0 << 0 << 1 << arma::endr;

	if(DEBUG_HOMOGRAPHY==true)
	{
	/*
		R(0,0) = 0;
		R(0,1) = -1.0;
		R(0,2) = 0;
		R(1,0) = 1.0;
		R(1,1) = 0;
		R(1,2) = 0;
		R(2,0) = 0;
		R(2,1) = 0;
		R(2,2) = 1.0;
	
		H = R+u*v.st();	
	*/
	}
	//else
	//{
		//Convert cv::Mat to arma::mat
		H(0,0) = homography.at<double>(0,0);
		H(0,1) = homography.at<double>(0,1);
		H(0,2) = homography.at<double>(0,2);
		H(1,0) = homography.at<double>(1,0);
		H(1,1) = homography.at<double>(1,1);
		H(1,2) = homography.at<double>(1,2);
		H(2,0) = homography.at<double>(2,0);
		H(2,1) = homography.at<double>(2,1);
		H(2,2) = homography.at<double>(2,2);
	//}
	
	//Algorithm
	arma::vec m_val = arma::svd(H);
	double gamma = arma::median(m_val);
	H = H/gamma;
	
	arma::mat S = H.st()*H-I;
	//S.print("S =");

	int size = 3;
	for (int q=0;q<size;q++)
		for (int p=0;p<size;p++)
		{
     		int m=0;
     		int n=0;
     		for (int i=0;i<size;i++)
      			for (int j=0;j<size;j++)
        		{
          			if (i!=q && j!=p)
          			{
            			temp(m,n)=S(i,j);
            			if (n<(size-2)) n++;
            			else
             			{
              				n=0;
               				m++;
               			}

            		}
        		}
        	//temp.print("Temp =");
        	M(q,p)=-arma::det(temp);
        }

	na1 << S(0,0) << S(0,1) + sqrt(M(2,2)) << S(0,2) + sgn(M(1,2))*sqrt(M(1,1)) << arma::endr;
	na1 = na1/arma::norm(na1);
	na2 << S(0,1)+sqrt(M(2,2)) << S(1,1) <<	S(1,2)-sgn(M(0,2))*sqrt(M(0,0)) << arma::endr;
	na2 = na2/arma::norm(na2);
	na3 << S(0,2)+sgn(M(0,1))*sqrt(M(1,1)) << S(1,2)+sqrt(M(0,0)) << S(2,2) << arma::endr;
	na3 = na3/arma::norm(na3);
	nb1 << S(0,0) << S(0,1)-sqrt(M(2,2)) << S(0,2)-sgn(M(1,2))*sqrt(M(1,1)) << arma::endr;
	nb1 = nb1/arma::norm(nb1);
	nb2 << S(0,1)-sqrt(M(2,2)) << S(1,1) <<	S(1,2)+sgn(M(0,2))*sqrt(M(0,0)) << arma::endr;
	nb2 = nb2/arma::norm(nb2);
	nb3 << S(0,2)-sgn(M(0,1))*sqrt(M(1,1)) << S(1,2)-sqrt(M(0,0)) << S(2,2) << arma::endr;
	nb3 = nb3/norm(nb3);
	
	double V = sqrt(2*((1+arma::trace(S))*(1+arma::trace(S))+1-trace(S*S)));
	es << sgn(S(0,0)) << sgn(S(1,1)) << sgn(S(2,2)) << arma::endr;
	double te = sqrt(2+arma::trace(S)-V);
	double p = sqrt(2+arma::trace(S)+V);
	
	
	if(norm(na1-nb1) > 0.0001)
	{
		ta = (te/2)*(es(0)*p*nb1-te*na1);
		tb = (te/2)*(es(0)*p*na1-te*nb1);
		if(na1(2) > nb1(2))
		{
			R = H*(I-(2/V)*ta*na1.t());
			R2 = H*(I-(2/V)*tb*nb1.t());
		}
		else
		{
			R = H*(I-(2/V)*tb*nb1.t());
			R2 = H*(I-(2/V)*ta*na1.t());
		}
	}
	else if (norm(na2-nb2) > 0.0001)
	{
		ta = (te/2)*(es(1)*p*nb2-te*na2);
		tb = (te/2)*(es(1)*p*na2-te*nb2);
		if(na2(2) > nb2(2))
		{
			R = H*(I-(2/V)*ta*na2.t());
			R2 = H*(I-(2/V)*tb*nb2.t());
		}
		else
		{
			R = H*(I-(2/V)*tb*nb2.t());
			R2 = H*(I-(2/V)*ta*na2.t());
		}
	}
	else if (norm(na3-nb3) > 0.0001)
	{
		ta = (te/2)*(es(2)*p*nb3-te*na3);
		tb = (te/2)*(es(2)*p*na3-te*nb3);
		if(na3(2) > nb3(2))
		{
			R = H*(I-(2/V)*ta*na3.t());
			R2 = H*(I-(2/V)*tb*nb3.t());
		}
		else
		{
			R = H*(I-(2/V)*tb*nb3.t());
			R2 = H*(I-(2/V)*ta*na3.t());
		}
	}
	
	if(DEBUG_HOMOGRAPHY==true)
	{
		arma::vec ta1(3), ta2(3), ta3(3);
		arma::vec tb1(3), tb2(3), tb3(3);
		arma::mat Ra1(3,3), Ra2(3,3), Ra3(3,3);
		arma::mat Rb1(3,3), Rb2(3,3), Rb3(3,3);
		
		na1.print("na1 =");
		na2.print("na2 =");
		na3.print("na3 =");
		nb1.print("nb1 =");
		nb2.print("nb2 =");
		nb3.print("nb3 =");
	
		ta1 = (te/2)*(es(0)*p*nb1-te*na1);
		ta2 = (te/2)*(es(1)*p*nb2-te*na2);
		ta3 = (te/2)*(es(2)*p*nb3-te*na3);
		tb1 = (te/2)*(es(0)*p*na1-te*nb1);
		tb2 = (te/2)*(es(1)*p*na2-te*nb2);
		tb3 = (te/2)*(es(2)*p*na3-te*nb3);
	
		Ra1 = H*(I-(2/V)*ta1*na1.t());
		Ra2 = H*(I-(2/V)*ta2*na2.t());
		Ra3 = H*(I-(2/V)*ta3*na3.t());
		Rb1 = H*(I-(2/V)*tb1*nb1.t());
		Rb2 = H*(I-(2/V)*tb2*nb2.t());
		Rb3 = H*(I-(2/V)*tb3*nb3.t());
	
		Ra1.print("Ra1 =");
		Ra2.print("Ra2 =");
		Ra3.print("Ra3 =");
		Rb1.print("Rb1 =");
		Rb2.print("Rb2 =");
		Rb3.print("Rb3 =");
		
		H.print("H = ");
		M.print("M = ");

	}
	ta.print("ta = ");
	tb.print("tb = ");
	R.print("R = ");
	R2.print("R2 = ");
}

//returns 1 for positive and 0 values, and -1 for negative
template <typename T> int sgn(T val) {
	if((T(0) < val) - (val < T(0)) == 0) return 1;
    else return (T(0) < val) - (val < T(0));
}
