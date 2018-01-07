#ifndef COORDINATEGEOMETRYCALCULATIONS_H_INCLUDE
#define COORDINATEGEOMETRYCALCULATIONS_H_INCLUDE
 
#define EPSILON_RADIUS_FROM_TARGET			 0.75

#include "math.h"
#include "geometry_msgs/Point.h"

bool isRadianInZeroInterval(double *value)
{
	return ((*value < 0.1) && (*value > -0.1));
};

bool isDroneInRadius(geometry_msgs::Point *dronePosition, geometry_msgs::Point *destPosition)
{
	if ( pow(dronePosition->x - destPosition->x, 2) + pow(dronePosition->y - destPosition->y, 2) <= pow(EPSILON_RADIUS_FROM_TARGET, 2) )
		return true;

	return false;
}

double radToDegree(double angleInRad)
{
	return (angleInRad * (180.0/M_PI));
}

/* Your function statement here */
#endif
