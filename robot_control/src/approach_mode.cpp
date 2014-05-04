#include "approach_mode.h"
#include <stdint.h>
#include <math.h>

int Approach_Class::drive(int stop, Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	const int speed = 500; // 50% speed
	end_stage[0] = actuators.drive.driveStraightTrigger(speed,0,stop,&robot_status);
	end_stage[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators.gimbal.rotateGimbal(0.0,0.0,0.0,&robot_status);
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) return 1;
	else return 0;
}

int Approach_Class::pivot(float desired_heading, Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	const int speed = 500; // 50% speed
	end_stage[0] = actuators.drive.pivotAbsolute(speed,desired_heading,&robot_status);
	end_stage[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators.gimbal.rotateGimbal(0.0,0.0,0.0,&robot_status);
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) return 1;
	else return 0;
}

int Approach_Class::run(float desired_heading, Actuators* actuators, Robot_Status* robot_status)
{
	static int stage = 1;
	int mode_ended = 0;
	int stage_ended = 0;
	switch(stage)
	{
		case 1: // Pivot to face object
			stage_ended = pivot(desired_heading,&actuators,&robot_status);
			mode_ended = 0;
			if(stage_ended) stage = 2;
			else stage = 1;
			break;
		case 2: // Drive up to object
			stage_ended = drive(robot_status.object_seen,&actuators,&robot_status);
			mode_ended = 0;
			if(stage_ended) stage = 3;
			else stage = 2;
			break;
		case 3: // end mode
			actuators.drive.stop();
			actuators.gimbal.rotateGimbal(0.0,0.0,0.0,&robot_status);
			mode_ended = 1;
			stage = 1;
			break;
	}
	return mode_ended;
}
