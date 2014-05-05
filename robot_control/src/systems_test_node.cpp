#include <ros/ros.h>
#include <stdint.h>
#include <math.h>
#include "robot_actuators_class.h"
#include <roboteq_interface/motor_commands.h>
#include <serial_comm/pkt_7_msg.h>
#include <serial_comm/pkt_5_msg.h>

using namespace std;

Actuators actuators;
Robot_Status robot_status;

void packMsg();
void sibCallback(const serial_comm::pkt_5_msg::ConstPtr& sib_msg_in);

roboteq_interface::motor_commands motor_msg_out;
serial_comm::pkt_7_msg sib_msg_out;

int main(int argc, char **argv)
{
	int stop;
	int counter = 0;
	int stage_ended;
    // Node Initialization
    ros::init(argc, argv, "systems_test_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    ros::Publisher motor_pub = nh.advertise<roboteq_interface::motor_commands>("motor_speed_control",1);
    ros::Publisher sib_pub = nh.advertise<serial_comm::pkt_7_msg>("mis/sib_out_data",1);
    ros::Subscriber sib_sub = nh.subscribe<serial_comm::pkt_5_msg>("mis/sib_in_data",1,sibCallback);
    
    while(ros::ok())
    {
    	if(counter>500) robot_status.object_seen = 1;
    	else robot_status.object_seen = 0;
    	//robot_status.heading = -counter;
    	//stop = 0;
    	stage_ended = actuators.gimbal.scan(0,&robot_status);
    	//actuators.drive.driveStraightTrigger(500,0, 0, &robot_status);
    	cout << stage_ended << endl;
    	counter++;
		packMsg();
		motor_pub.publish(motor_msg_out);
		sib_pub.publish(sib_msg_out); 
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}

void sibCallback(const serial_comm::pkt_5_msg::ConstPtr& sib_msg_in)
{
	if(sib_msg_in->Stepper < 32768) robot_status.gimbal_yaw_angle = sib_msg_in->Stepper/10.0;
	else robot_status.gimbal_yaw_angle = (sib_msg_in->Stepper - 65536)/10.0;
}

void packMsg()
{
	if(actuators.drive.front_left_motor_speed>1000) actuators.drive.front_left_motor_speed = 1000;
	else if(actuators.drive.front_left_motor_speed<-1000) actuators.drive.front_left_motor_speed = -1000;
	if(actuators.drive.rear_left_motor_speed>1000) actuators.drive.rear_left_motor_speed = 1000;
	else if(actuators.drive.rear_left_motor_speed<-1000) actuators.drive.rear_left_motor_speed = -1000;
	if(actuators.drive.front_right_motor_speed>1000) actuators.drive.front_right_motor_speed = 1000;
	else if(actuators.drive.front_right_motor_speed<-1000) actuators.drive.front_right_motor_speed = -1000;
	if(actuators.drive.rear_right_motor_speed>1000) actuators.drive.rear_right_motor_speed = 1000;
	else if(actuators.drive.rear_right_motor_speed<-1000) actuators.drive.rear_right_motor_speed = -1000;
	motor_msg_out.cont_1_motor_1_speed_cmd = actuators.drive.front_left_motor_speed;
	motor_msg_out.cont_1_motor_2_speed_cmd = actuators.drive.rear_left_motor_speed;
	motor_msg_out.cont_2_motor_1_speed_cmd = actuators.drive.front_right_motor_speed;
	motor_msg_out.cont_2_motor_2_speed_cmd = actuators.drive.rear_right_motor_speed;
	if(actuators.gimbal.pitch_angle > actuators.gimbal.gimbal_max_angle) actuators.gimbal.pitch_angle = actuators.gimbal.gimbal_max_angle;
	else if(actuators.gimbal.pitch_angle < actuators.gimbal.gimbal_min_angle) actuators.gimbal.pitch_angle = actuators.gimbal.gimbal_min_angle;
	if(actuators.gimbal.roll_angle > actuators.gimbal.gimbal_max_angle) actuators.gimbal.roll_angle = actuators.gimbal.gimbal_max_angle;
	else if(actuators.gimbal.roll_angle < actuators.gimbal.gimbal_min_angle) actuators.gimbal.roll_angle = actuators.gimbal.gimbal_min_angle;
	if(actuators.gimbal.yaw_angle > actuators.gimbal.gimbal_max_angle) actuators.gimbal.yaw_angle = actuators.gimbal.gimbal_max_angle;
	else if(actuators.gimbal.yaw_angle < actuators.gimbal.gimbal_min_angle) actuators.gimbal.yaw_angle = actuators.gimbal.gimbal_min_angle;
	sib_msg_out.Desired_Angle_1 = round(actuators.gimbal.pitch_angle*10);
	sib_msg_out.Desired_Angle_2 = round(actuators.gimbal.roll_angle*10);
	sib_msg_out.Desired_Angle_3 = round(actuators.gimbal.yaw_angle*10);
}