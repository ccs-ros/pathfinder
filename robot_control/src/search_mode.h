#include <stdint.h>
#include <math.h>
#include "robot_actuators_class.h"
#include "robot_status_class.h"

class Spiral
{
public:
	static float gimbal_target_yaw_angle = 0.0;
	int drive(float distance, Actuators* actuators, Robot_Status* robot_status);
	int pivot(float desired_angle, Actuators* actuators, Robot_Status* robot_status);
	int scan(int object_seen, Actuators* actuators, Robot_Status* robot_status);
	int run(Actuators& actuators, Robot_Status* robot_status);
};

class Search_Class(
{
public:
	Spiral spiral;
};
