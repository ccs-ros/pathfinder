#include <stdint.h>
#include <math.h>

class Approach_Class
{
public:
	int drive(int stop, Actuators* actuators, Robot_Status* robot_status);
	int pivot(float desired_heading, Actuators* actuators, Robot_Status* robot_status);
	int run(float desired_heading, Actuators* actuators, Robot_Status* robot_status);
};
