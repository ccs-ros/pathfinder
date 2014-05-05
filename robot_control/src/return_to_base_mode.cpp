#include <stdint.h>
#include <math.h>
#include "robot_actuators_class.h"
#include "return_to_base_mode.h"

const float PI = 3.14159;

float Return_to_base_Class::gimbal_target_yaw_angle = 0.0;

int Return_to_base_Class::drive(int stop, Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	const int speed = 500; // 50% speed
	static int conclude = 0;
	int return_value;
	end_stage[0] = actuators->drive.driveStraightTrigger(speed,0,stop,conclude,robot_status);
	end_stage[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators->gimbal.rotateGimbal(0.0,0.0,0.0,conclude,robot_status);
	if(conclude == 1) return_value = 1;
	else return_value = 0;
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) conclude = 1;
	else conclude = 0;
	return return_value;
}

int Return_to_base_Class::pivot(float desired_heading, Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	const int speed = 500; // 50% speed
	static int conclude = 0;
	int return_value;
	end_stage[0] = actuators->drive.pivotAbsolute(speed,desired_heading,conclude,robot_status);
	end_stage[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators->gimbal.rotateGimbal(0.0,0.0,0.0,conclude,robot_status);
	if(conclude == 1) return_value = 1;
	else return_value = 0;
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) conclude = 1;
	else conclude = 0;
	return return_value;
}
int Return_to_base_Class::scan(Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	static int conclude = 0;
	int return_value;
	end_stage[0] = actuators->drive.stop();
	end_stage[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators->gimbal.scan(1,conclude,robot_status);
	if(conclude == 1) return_value = 1;
	else return_value = 0;
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) conclude = 1;
	else conclude = 0;
	return return_value;
}

int Return_to_base_Class::run(float desired_heading, Actuators* actuators, Robot_Status* robot_status)
{
	static int stage = 1;
	int mode_ended = 0;
	int stage_ended = 0;
	switch(stage)
	{
		case 1: // Scan for homing beacon
			stage_ended = scan(actuators,robot_status);
			if(robot_status->beacon_seen==1) gimbal_target_yaw_angle = -robot_status->gimbal_yaw_angle+robot_status->heading*180.0/PI;
			mode_ended = 0;
			if(stage_ended) stage = 2;
			else stage = 1;
			break;
		case 2: // Pivot to face homing beacon
			stage_ended = pivot(desired_heading,actuators,robot_status);
			mode_ended = 0;
			if(stage_ended) stage = 3;
			else stage = 2;
			break;
		case 3: // Drive up to homing beacon
			stage_ended = drive(robot_status->beacon_in_range,actuators,robot_status);
			mode_ended = 0;
			if(stage_ended) stage = 4;
			else stage = 3;
			break;
		case 4: // End Mode
			actuators->drive.stop();
			actuators->gimbal.rotateGimbal(0.0,0.0,0.0,1,robot_status);
			mode_ended = 1;
			stage = 1;
			break;
	}
	return mode_ended;
}
