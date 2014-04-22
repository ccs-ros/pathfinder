/**
 * Author: Jared Strader
 * Last Update: 3/27/14 
 *
 * This node subscribes to the det/cam_select message to receive the desired state of the
 * camera. The node uses fork() and system() to turn on or off the cameras and monitor 
 * the states. The launch files must only launch one node and must be named according to
 * the details given in the notes below. 
 *
 * NOTES: Launch commands executed are -> roslaunch computer_vision gscam_high.launch (/dev/video0)
 *				       -> roslaunch computer_vision gscam_low.launch (/dev/video1)
 *	  
 *	  The node terminted is called -> "/logitech_c920"
 */

#include <ros/ros.h>
#include <computer_vision/Camera.h>
#include <signal.h>   //SIGTERM
#include <sys/wait.h> //waitpid

using namespace std;

const unsigned int SWITCH_TO_HIGH = 1;
const unsigned int SWITCH_TO_LOW = -1;
const unsigned int HIGH_CAMERA = 1;
const unsigned int LOW_CAMERA = 2;

void killChild(pid_t childpid); //kill child process

class Selector //Class containing callback function to det/cam_select
{
public:
  ros::Subscriber sub;
  unsigned int camera;

  Selector() : camera(1)
  {
    ros::NodeHandle node;
    sub = node.subscribe("det/cam_select", 1, &Selector::camSelectCallback, this);
  }

  void camSelectCallback(const computer_vision::Camera::ConstPtr &msg) {
    this->camera = msg->camera;
  }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "det_select_node");
    Selector sel;

    //Camera variables
    unsigned int pastCam = 1, camSelect=1, camSwitch=1;

    //Child process pid
    pid_t childpid, endpid;
    int status;
	
    while(ros::ok())
    {
	ROS_INFO("cam_select = %i", sel.camera);
	
	if(camSwitch>0) 
	{
	    childpid = fork();
	    camSwitch = 0; //reset camera select switch
	}

	if(childpid == 0) //only child process
	{
	    if(camSelect==HIGH_CAMERA) system("roslaunch computer_vision gscam_high.launch");
	    if(camSelect==LOW_CAMERA) system("roslaunch computer_vision gscam_low.launch");
	    sleep(1);
	}
	else if (childpid<0) //failed to fork
	{
	    ROS_ERROR("Child process for cam_select_node failed to open!");
	}
	else  //only parent process
	{   
	   if(sel.camera-pastCam==SWITCH_TO_LOW) 
	   {
		killChild(childpid); //change to low camera
		camSelect=LOW_CAMERA;
		camSwitch=1;
	   }
	   else if(sel.camera-pastCam==SWITCH_TO_HIGH) 
	   {
		killChild(childpid); //change to high camera
		camSelect=HIGH_CAMERA;
		camSwitch=1;
	   }

	   sleep(1);
	}
	
	//save previous camera setting
	pastCam=sel.camera;

	ros::spinOnce();
	sleep(0.1);
    }

    return 0;
}

void killChild(pid_t childpid)
{
    pid_t endpid;
    int status;

    system("rosnode kill /logitch_c920"); //kill camera node
    ROS_INFO("/logitech_c920 shutdown complete.");

    kill(childpid, SIGTERM); //send kill command to child process
    
   for(int i=0; i<15; i++) //give 15 seconds for child process to terminate
   {
	endpid=waitpid(childpid, &status, WNOHANG|WUNTRACED); //check for return status

	if(endpid==0) //child process still running
	{
		ROS_INFO("det_select_node waiting for child process to end.");
	}
	else if(endpid==childpid) //child process ended
	{
	    if (WIFEXITED(status)) ROS_INFO("Child process ended normally.");
            else if (WIFSIGNALED(status)) ROS_INFO("Child process ended due to uncaught signal.");
            else if (WIFSTOPPED(status)) ROS_INFO("Child process stopped.");
	    
	    ROS_INFO("Process det_select_node terminated!");
	    break;
	}
	else if(endpid==-1) //child process exit with error
	{
		ROS_WARN("det_select_node child process exit with status -1.");
		break;
	}

	sleep(1); //wait a second
   }
}
