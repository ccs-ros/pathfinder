#include "ros/ros.h"
#include <sstream>
#include <stdint.h>
#include "roboteq_interface/motor_speed_control.h"
#include "roboteq_interface/left_motor_encoders.h"
#include "roboteq_interface/right_motor_encoders.h"
#include <math.h>

#define PI 3.14159

using namespace ros;

int16_t front_left_motor_speed;
int16_t front_right_motor_speed;
int16_t rear_left_motor_speed;
int16_t rear_right_motor_speed;
int16_t front_left_encoder_count;
int16_t front_right_encoder_count;
int16_t rear_left_encoder_count;
int16_t rear_right_encoder_count;
const double wheel_radius = 0.05; // meters
double front_left_encoder_distance;
double front_right_encoder_distance;
static double front_right_encoder_distance_prev;
double rear_left_encoder_distance;
double rear_right_encoder_distance;
const double wheel_base_radius = 0.3175; // meters
const double front_left_encoder_gain =  4.203*pow(10,-6); // meters/count
const double front_right_encoder_gain = 4.203*pow(10,-6); // meters/count
const double rear_left_encoder_gain =   4.203*pow(10,-6); // meters/count
const double rear_right_encoder_gain =  4.203*pow(10,-6); // meters/count

/* publisher message object */
roboteq_interface::motor_speed_control msg_out;

void leftMotorCallback(const roboteq_interface::left_motor_encoders::ConstPtr& left_msg_in);
void rightMotorCallback(const roboteq_interface::right_motor_encoders::ConstPtr& right_msg_in);
void packControlMsg();
void setPivotCCW();
void setDriveStraight();
void setNoOp();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_encoder_test");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roboteq_interface::motor_speed_control>("motor_speed_control",1000);
  ros::Subscriber left_sub = n.subscribe<roboteq_interface::left_motor_encoders>("left_roboteq_encoders", 1000, leftMotorCallback);
  ros::Subscriber right_sub = n.subscribe<roboteq_interface::right_motor_encoders>("right_roboteq_encoders", 1000, rightMotorCallback);
  ros::Rate loop_rate(2);

  int step = 0;
  int startup = 0;
  bool first_pass = true;

  while (ros::ok())
  {
    switch (step)
    {
        case 0: // hold position (drive 0 m)
          if (first_pass == true)
          {
            setNoOp();
            pub.publish(msg_out);
            first_pass = false;
            ROS_INFO("%s","No Op sent");
          }
          if (startup >= 5)
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
          startup++;
          pub.publish(msg_out);  //~~~test pub
          ROS_INFO("%s","Case 0");
          break;
        case 1: // drive straight 1 m
          if (first_pass == true)
          {
            setDriveStraight();
            pub.publish(msg_out);
            first_pass = false;
            ROS_INFO("%s","First Call");
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev) >= (1.0))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
//          pub.publish(msg_out); //~~~ test pubs
          ROS_INFO("%s","Case 1");
	  ROS_INFO("%f",front_right_encoder_distance);
	  ROS_INFO("%f",front_right_encoder_distance_prev);
          break;
        case 2: // Turn 90 Degrees
          if (first_pass == true)
          {
            setPivotCCW();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev)/wheel_base_radius >= (PI/2))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 2");
          break;
        case 3: // drive straight 1 m
          if (first_pass == true)
          {
            setDriveStraight();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev) >= (1.0))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 3");
          break;
        case 4: // Turn 90 Degrees
          if (first_pass == true)
          {
            setPivotCCW();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev)/wheel_base_radius >= (PI/2-5*PI/180))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 4");
          break;
        case 5: // drive straight 1 m
          if (first_pass == true)
          {
            setDriveStraight();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev) >= (1.0))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 5");
          break;
        case 6: // Turn 90 Degrees
          if (first_pass == true)
          {
            setPivotCCW();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev)/wheel_base_radius >= (PI/2))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 6");
          break;
        case 7: // drive straight 1 m
          if (first_pass == true)
          {
            setDriveStraight();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev) >= (1.0))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 7");
          break;
        case 8: // Turn 90 Degrees
          if (first_pass == true)
          {
            setPivotCCW();
            pub.publish(msg_out);
            first_pass = false;
          }
          if (fabs(front_right_encoder_distance-front_right_encoder_distance_prev)/wheel_base_radius >= (PI/2))
          {
            first_pass = true;
	    front_right_encoder_distance_prev = front_right_encoder_distance;
            step++;
          }
	  ROS_INFO("%s","Case 8");
          break;
        default:
	  setNoOp();
          pub.publish(msg_out);
          break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void leftMotorCallback(const roboteq_interface::left_motor_encoders::ConstPtr& left_msg_in)
{
  //front_right_encoder_distance =  left_msg_in->cont_2_motor_1_encoder_count*front_right_encoder_gain;
}

void rightMotorCallback(const roboteq_interface::right_motor_encoders::ConstPtr& right_msg_in)
{
  front_right_encoder_distance =  right_msg_in->motor_1_encoder_count*front_right_encoder_gain;
}

void packControlMsg()
{
  msg_out.cont_1_motor_1_speed_cmd = front_left_motor_speed;
  msg_out.cont_1_motor_2_speed_cmd = rear_left_motor_speed;
  msg_out.cont_2_motor_1_speed_cmd = front_right_motor_speed;
  msg_out.cont_2_motor_2_speed_cmd = rear_right_motor_speed;
}

void setNoOp()
{
  front_left_motor_speed  = 0.0;
  front_right_motor_speed = 0.0;
  rear_left_motor_speed   = 0.0;
  rear_right_motor_speed  = 0.0;
  packControlMsg();
}

void setPivotCCW()
{
  front_left_motor_speed = -500;
  front_right_motor_speed = 500;
  rear_left_motor_speed = -500;
  rear_right_motor_speed = 500;
  packControlMsg();
}

void setDriveStraight()
{
  front_left_motor_speed  = 500;
  front_right_motor_speed = 500;
  rear_left_motor_speed   = 500;
  rear_right_motor_speed  = 500;
  packControlMsg();
}
