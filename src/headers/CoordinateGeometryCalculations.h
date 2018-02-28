#ifndef COORDINATEGEOMETRYCALCULATIONS_H_INCLUDE
#define COORDINATEGEOMETRYCALCULATIONS_H_INCLUDE
 
#define EPSILON_RADIUS_FROM_TARGET			 0.6

#include "math.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

class Circle
{
	public:
			geometry_msgs::Point centre;
			double radius;
			
	Circle(geometry_msgs::Point _centre, double _radius) : centre(_centre), radius(_radius) {}
};


//-------------------------------------------vector operations---------------------------------------------------

geometry_msgs::Vector3 createVector(geometry_msgs::Point &A, geometry_msgs::Point &B)
{
	geometry_msgs::Vector3 newVec;
	newVec.x = B.x - A.x;
	newVec.y = B.y - A.y;
	newVec.z = B.z - A.z;

	return newVec;
}


double scalarProduct2D(geometry_msgs::Vector3 &a, geometry_msgs::Vector3 &b)
{
    return (a.x*b.x + a.y*b.y);
}


geometry_msgs::Vector3 getPerpendicularVector(geometry_msgs::Vector3 &base, geometry_msgs::Point &baseEnd, geometry_msgs::Vector3 &resultVec,
											  geometry_msgs::Point &C, geometry_msgs::Point &D)
{
	geometry_msgs::Vector3 BC = createVector(baseEnd, C);
	geometry_msgs::Vector3 BD = createVector(baseEnd, D);
	double scalar1 = fabs(scalarProduct2D(base, BC));
	double scalar2 = fabs(scalarProduct2D(base, BD));

	resultVec = (scalar1 < scalar2) ? BC : BD;
}


//----------------------------------------------map operations---------------------------------------------------

bool isPointInsideCorridor(geometry_msgs::Point &testPoint, geometry_msgs::Vector3 &AB, geometry_msgs::Vector3 &BC,
							geometry_msgs::Point &A, geometry_msgs::Point &B, geometry_msgs::Point &C)
{
	geometry_msgs::Vector3 AM = createVector(A, testPoint);
	geometry_msgs::Vector3 BM = createVector(B, testPoint);

	double dotABAB = scalarProduct2D(AB, AB);
	double dotBCBC = scalarProduct2D(BC, BC);
	double dotABAM = scalarProduct2D(AB, AM);
	double dotBCBM = scalarProduct2D(BC, BM);

	return (0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC);
}


int intersectionPointsOfCircles(geometry_msgs::Point centre1, double radius1, 
								geometry_msgs::Point centre2, double radius2, 
								geometry_msgs::Point *intersectionPoint1, geometry_msgs::Point *intersectionPoint2)
{
	Circle c1(centre1, radius1);
	Circle c2(centre2, radius2);

	double a, dx, dy, d, h, rx, ry;
	double x2, y2;

	dx = c2.centre.x - c1.centre.x;
	dy = c2.centre.y - c1.centre.y;
	d = sqrt( pow(dy, 2) + pow(dx, 2) );

	if (d > (c1.radius + c2.radius))
	{
		return 0;
	}
	if (d < fabs(c1.radius - c2.radius))
	{
		return 0;
	}

	a = ( pow(c1.radius, 2) - pow(c2.radius, 2) + pow(d, 2) ) / (2.0 * d) ;
	x2 = c1.centre.x + (dx * a/d);
	y2 = c1.centre.y + (dy * a/d);
	h = sqrt( pow(c1.radius, 2) - pow(a, 2) );
	rx = -dy * (h/d);
	ry = dx * (h/d);

	intersectionPoint1->x = x2 + rx;
	intersectionPoint1->y = y2 + ry;

	intersectionPoint2->x  = x2 - rx;
	intersectionPoint2->y = y2 - ry;

	return 2;
}


//------------------------------------------general operations---------------------------------------------------

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
