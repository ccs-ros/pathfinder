#include <stdint.h>
#include <math.h>
#include "robot_actuators_class.h"

class Return_to_base_Class
{
public:
	float gimbal_target_yaw_angle;
	int drive(int stop, Actuators* actuators, Robot_Status* robot_status);
	int pivot(float desired_heading, Actuators* actuators, Robot_Status* robot_status);
	int scan(Actuators* actuators, Robot_Status* robot_status);
	int run(float desired_heading, Actuators* actuators, Robot_Status* robot_status);
};
