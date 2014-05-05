#include <stdint.h>
#include <math.h>

class Spiral
{
public:
	static float gimbal_target_yaw_angle;
	int drive(float distance, Actuators* actuators, Robot_Status* robot_status);
	int pivot(float desired_angle, Actuators* actuators, Robot_Status* robot_status);
	int scan(Actuators* actuators, Robot_Status* robot_status);
	int run(Actuators* actuators, Robot_Status* robot_status);
};

class Search_Class
{
public:
	Spiral spiral;
};
