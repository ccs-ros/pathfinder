#include "robot_actions_class.h"
#include <math.h>
#include <iostream>

float Robot_Actions::calculateHeading(*waypoint_type* waypoint, Robot_Status* robot_status)
{
	float waypoint_x;
	float waypoint_y;
	float current_x;
	float current_y;
	float new_heading;
	float x_err;
	float y_err;
	float u_x_act;
	float u_y_act;
	float u_x_des;
	float u_y_des;
	float new_heading_sign;
	waypoint_x waypoint[0]*cos(waypoint[1]*PI/180);
	waypoint_y waypoint[0]*sin(waypoint[1]*PI/180);
	current_x = robot_status.polar_distance*cos(robot_status.bearing);
	current_y = robot_status.polar_distance*sin(robot_status.bearing);
	x_err = waypoint_x-current_x;
	y_err = waypoint_y-current_y;
	u_x_des = x_err/hypot(x_err,y_err);
	u_y_des = y_err/hypot(x_err,y_err);
	u_x_act = cos(robot_status.heading);
	u_y_act = sin(robot_status.heading);
	if(asin(u_x_act*u_y_des-u_x_des*u_y_act)>=0 new_heading_sign = 1.0;
	else new_heading_sign = -1.0;
	new_heading = acos(u_x_act*u_x_des+u_y_act*u_y_des);
	return new_heading;
}
	
int Robot_Actions::travelWP(float waypoint[2,1], Actuators* actuators, Robot_Status* robot_status)
{
	float pivot_heading;
	float waypoint_x;
	float waypoint_y;
	float current_x;
	float current_y;
	static int step = 1;
	int step_ended;
	int task_ended[3];
	int end_task;
	int waypoint_reached;
	const float waypoint_distance_tolerance = 1.0; // m
	float distance_to_waypoint;
	switch(step)
	{
		case 1:
			task_ended = 0;
			pivot_heading = calculateHeading(waypoint,&robot_status);
			step_ended = actuators.drive.pivotAbsolute(500,pivot_heading,&robot_status);
			task_ended[0] = step_ended;
			task_ended[1] = 1; // Placeholder for grabber
			task_ended[2] = actuators.gimbal.hold(&robot_status);
			if(step_ended==1) step = 2;
			else step = 1;
			break;
		case 2:
			waypoint_x waypoint[0,1]*cos(waypoint[1,1]*PI/180);
			waypoint_y waypoint[0,1]*sin(waypoint[1,1]*PI/180);
			current_x = robot_status.polar_distance*cos(robot_status.bearing);
			current_y = robot_status.polar_distance*sin(robot_status.bearing);
			pivot_heading = calculateHeading(waypoint,&robot_status);
			task_ended = 0;
			if(hypot((waypoint_x-current_x),(waypoint_y-current_y))<waypoint_distance_tolerance) waypoint_reached = 1;
			else waypoint_reached = 0;
			step_ended = actuators.drive.driveStraightTriggerHeading(500,0,waypoint_reached,pivot_heading,&robot_status);
			task_ended[0] = step_ended;
			task_ended[1] = 1; // Placeholder for grabber
			task_ended[2] = actuators.gimbal.hold(&robot_status);
			if(step_ended==1) step = 3;
			else step = 2;
			break;
		case 3:
			actuators.drive.stop();
			actuators.gimbal.hold(&robot_status);
			task_ended = 1;
			step = 1;
			break;
	}
	if((task_ended[0]==1)&&(task_ended[1]==1)&&(task_ended[2]==1)) end_task = 1;
	else end_task = 0;
	return end_task;
}

int Robot_Actions::returnHome(Actuators* actuators, Robot_Status* robot_status)
{
	
