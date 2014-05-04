#include <stdint.h>
#include <math.h>
#include "search_mode.h"
#include "robot_status_class.h"

int Sprial::drive(float distance, Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	const int speed = 500; // 50% speed
	int stop;
	if(robot_status.polar_distance>=distance) stop = 1;
	else stop = 0;
	end_stage[0] = actuators.drive.driveStraightTrigger(speed,0,stop,&robot_status);
	end_state[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators.gimbal.rotateGimbal(0.0,0.0,0.0,&robot_status);
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) return 1;
	else return 0;
}

int Spiral::pivot(float desired_angle, Actuators* actuators, Robot_Status* robot_status)
{
	int end_stage [3];
	const int speed = 500; // 50% speed
	end_stage[0] = actuators.drive.pivotDelta(speed,desired_angle,&robot_status);
	end_state[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators.gimbal.rotateGimbal(0.0,0.0,0.0,&robot_status);
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) return 1;
	else return 0;
}

int Spiral::scan(Actuators* actuators, Robot_Status* robot_status)
{
	end_stage[0] = actuators.drive.stop();
	end_state[1] = 1; // Placeholder for grabber
	end_stage[2] = actuators.gimbal.scan(0,&robot_status);
	if(end_stage[0]&&end_stage[1]&&end_stage[2]) return 1;
	else return 0;
}

int Sprial::run(Actuators* actuators, Robot_Status* robot_status)
{
	static int stage = 1;
	int mode_ended = 0;
	int stage_ended = 0;
	switch(stage)
	{
		case 1: // Drive out to set distance
			stage_enged = drive(15.0,&actuators,&robot_status);
			mode_ended = 0;
			if(stage_ended) stage = 2;
			else stage = 1;
			break;
		case 2: // Scan for sample
			stage_ended = scan(&actuators,&robot_status);
			if(robot_status.object_seen==1) gimbal_target_yaw_angle = -robot_status.gimbal_yaw_angle+robot_status.heading;
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

