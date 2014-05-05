#include <stdint.h>
#include <math.h>
#include "robot_status_class.h"

enum substates {Init_Task, Exec_Task, End_Task};

class Drive
{
public:
	// Outputs; Range: [-1000,1000] positive is forward, negative is reverse
	int16_t front_left_motor_speed;
	int16_t middle_left_motor_speed;
	int16_t rear_left_motor_speed;
	int16_t front_right_motor_speed;
	int16_t middle_right_motor_speed;
	int16_t rear_right_motor_speed;
	// Tasks
	int stop(); // Stop all drive motors
	// Drive straight a distance specified in "desired_distance" while maintaining the initial heading. Positive distance is forward. Speed Range: [0,1000]
	int driveStraightDist(int speed, float desired_distance, int conclude, Robot_Status* robot_status);
	// Drive straight until stop = 1. Continue while stop = 0. Maintain the initial heading
	int driveStraightTrigger(int speed, int reverse, int stop, int conclude, Robot_Status* robot_status);
	// Drive straight until stop = 1. Continue while stop = 0. Maintain the specified heading. reverse = 1, drive in reverse. else, drive forward. Speed Range: [0,1000]
	int driveStraightTriggerHeading(int speed, int reverse, int stop, float desired_heading, int conclude, Robot_Status* robot_status);
	// Pivot to a desired absolte heading angle. Positive heading angles are positive. Speed Range: [0,1000]
	int pivotAbsolute(int speed, float desired_angle, int conclude, Robot_Status* robot_status);
	// Pivot to a desired delta heading angle. Positive heading angles are positive. Speed Range: [0,1000]
	int pivotDelta(int speed, float desired_delta_angle, int conclude, Robot_Status* robot_status);
	// Pivot until stop = 1. Continue while stop = 0. Speed Range: [0,1000]
	int pivotTrigger(int speed, int CCW, int stop, int conclude, Robot_Status* robot_status);
};

class Grabber
{
public:
	uint8_t grabber_state; // 1 = up and open; 2 = grab; 3 = deposit
	int grabberState(int state, Robot_Status* robot_status);
};


class Gimbal
{
public:
	static const float gimbal_max_angle = 135.0; // Degrees
	static const float gimbal_min_angle = -135.0; // Degrees
	float pitch_angle; // CCW is positive
	float roll_angle; // CCW is positive
	float yaw_angle; // CCW is positive
	int rotateGimbal(float desired_pitch_angle, float desired_roll_angle, float desired_yaw_angle, int conclude, Robot_Status* robot_status);
	int hold(Robot_Status* robot_status);
	int scan(int homing_beacon, int conclude, Robot_Status* robot_status);
};


class Actuators
{
public:
	Drive drive;
	Grabber grabber;
	Gimbal gimbal;
};