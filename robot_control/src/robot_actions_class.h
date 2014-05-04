#include "robot_actuators_class.h"
#include "robot_status_class.h"
#include <stdint.h>

const float PI = 3.14159;

class Robot_Actions
{
public:
	float waypoints[2,100];
	float calculateHeading(*waypoint_type* waypoint, Robot_Status* robot_status);
	int travelWP(*waypoint_type* waypoint, Actuators* actuators, Robot_Status* robot_status);
	int returnHome(Actuators* actuators, Robot_Status* robot_status);
