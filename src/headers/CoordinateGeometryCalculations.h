#ifndef COORDINATEGEOMETRYCALCULATIONS_H_INCLUDE
#define COORDINATEGEOMETRYCALCULATIONS_H_INCLUDE
 
#define EPSILON_RADIUS_FROM_TARGET			 0.6

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
};


double distanceFromTargetInMeter(geometry_msgs::Point *dronePosition, geometry_msgs::Point *destPosition)
{
	double distanceInMeter = sqrt(pow(dronePosition->x - destPosition->x, 2) + pow(dronePosition->y - destPosition->y, 2));
	return (distanceInMeter);
};


double distanceFromTargetInCm(geometry_msgs::Point *dronePosition, geometry_msgs::Point *destPosition)
{
	double distanceInMeter = sqrt(pow(dronePosition->x - destPosition->x, 2) + pow(dronePosition->y - destPosition->y, 2));
	return (distanceInMeter*100.0);
};


double radToDegree(double angleInRad)
{
	return (angleInRad * (180.0/M_PI));
};


double degreeToRad(double angleInDegree)
{
	return (angleInDegree * (M_PI/180.0));
};

/* Your function statement here */
#endif
