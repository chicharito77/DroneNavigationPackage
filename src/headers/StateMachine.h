#ifndef STATEMACHINE_H_INCLUDE
#define STATEMACHINE_H_INCLUDE

#define START								0x00    //states of the statemachine
#define HOVERING							0x01
#define TURN_RIGHT							0x02
#define TURN_LEFT							0x03
#define MOVE_TO_TARGET						0x04
#define TARGET_REACHED						0x05
#define EMERGENCY_LANDING                   0x06

#define DRONE_HEDGE_ADDRESS					  12

#define INPUT_DIST_MIN                         1.0  //macros for the linear velocity profile
#define INPUT_DIST_MAX                         3.5
#define OUTPUT_SPEED_MIN                       0.02
#define OUTPUT_SPEED_MAX                       0.1

#define SIDESPEED_MAX                          0.1  //macros for the PD control
#define KD_COEFFICIENT                         0.1
#define KD_TIME_CONSTANT                       0.066

#define LOGGING_MODULO                         8    //macro for the logging

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"

#include "DroneNavigationPackage/HedgePositions.h"
#include "CoordinateGeometryCalculations.h"


namespace StateActions
{
    geometry_msgs::Twist  turnMsg, turnMsg_2, hoveringMsg, calibrationMsg, moveForwardMsg;
    ros::Time start, finish;
    ros::Publisher movementTopic;
    ros::ServiceClient positionUpdater;
 
    double roll_starting, pitch_starting, yaw_starting, roll_current, pitch_current, yaw_current;
    double yaw_prev, yaw_goal, yaw_remainder;
    
    volatile int currentState;
    volatile bool turnToTarget = false;
    volatile bool turnedToMove = false;
    geometry_msgs::Point destinationCoordinate;
    geometry_msgs::Point rotationCentre;
    double ThetaZero, Theta, Distance;
    double currentDistance;

    //map and PD control data
    geometry_msgs::Point A, B, C;
    geometry_msgs::Vector3 AB, BC;
    double Kp, Kd;
    double actualDistanceFromVec, previousDistanceFromVec;
    double normalLengthOfST;

    int movementCtr;
    int helperCtr = 0;


    void setMovementValues(double *theta_zero, double *theta, double *distance, 
                            geometry_msgs::Point *_destinationCoordinate, geometry_msgs::Point *_rotCenter,
                            geometry_msgs::Point *a, geometry_msgs::Point *b, geometry_msgs::Point *c,
                            geometry_msgs::Vector3 *vec1, geometry_msgs::Vector3 *vec2, double *proportionalMember, double *derivativeMember)
    {
        ThetaZero = *theta_zero;
        Theta = *theta;
        Distance = *distance;
        destinationCoordinate.x = _destinationCoordinate->x;
        destinationCoordinate.y = _destinationCoordinate->y;
        destinationCoordinate.z = _destinationCoordinate->z;
        
        rotationCentre = *_rotCenter;
        A = *a;
        B = *b;
        C = *c;
        AB = *vec1;
        BC = *vec2;
        Kp = *proportionalMember;
        Kd = *derivativeMember;
    }


    void initializeMovementMessages()
    {
        hoveringMsg.linear.x = 0.0;
        hoveringMsg.linear.y = 0.0;
        hoveringMsg.linear.z = 0.0;
        hoveringMsg.angular.x = 0.0;
        hoveringMsg.angular.y = 0.0;
        hoveringMsg.angular.z = 0.0;

        turnMsg.angular.z = -0.1;
        turnMsg_2.angular.z = 0.2;
        calibrationMsg.linear.x = -0.1;    
        moveForwardMsg.linear.x = 0.08;
    }


    void adjustDroneSpeed(double distanceRemained, double distFromVec=DBL_MIN, double prevDistFromVec=DBL_MIN)
    {
        float value;
        if (distanceRemained <= INPUT_DIST_MIN)
			value = OUTPUT_SPEED_MIN;
		else if (distanceRemained >= INPUT_DIST_MAX)
			value = OUTPUT_SPEED_MAX;
        else
            value = ((distanceRemained - INPUT_DIST_MIN) * (OUTPUT_SPEED_MAX - OUTPUT_SPEED_MIN) / (INPUT_DIST_MAX - INPUT_DIST_MIN)) + OUTPUT_SPEED_MIN;
		

		StateActions::moveForwardMsg.linear.x = value;
        if (distFromVec != DBL_MIN) 
        {
            StateActions::moveForwardMsg.linear.y = (Kp * distFromVec) + ( Kd * ( distFromVec-prevDistFromVec ) / KD_TIME_CONSTANT );
        }
        else
        {
            StateActions::moveForwardMsg.linear.y = 0.0;
        }   
    }


    void Start_actions(tf::Quaternion quat)
    {
        movementTopic.publish(hoveringMsg);
    }

    void Hovering_actions(tf::Quaternion quat)
    {
        tf::Matrix3x3(quat).getRPY(roll_starting,pitch_starting,yaw_starting);
        movementTopic.publish(hoveringMsg);

        if (turnToTarget == true && turnedToMove == true)
        {
            start = ros::Time::now();
            currentState = MOVE_TO_TARGET;
            movementTopic.publish(moveForwardMsg);
        }
    }

    void TurnLeft_actions(tf::Quaternion quat)
    {     
        tf::Matrix3x3(quat).getRPY(roll_starting, pitch_starting, yaw_current);

        if (yaw_current == 0.000000)	
        {
            yaw_current = yaw_prev;
            ROS_INFO("Invalid yaw, change done.");                    
        }

        if ( fabs(yaw_prev) + fabs(yaw_current) > fabs(yaw_prev + yaw_current) && !isRadianInZeroInterval(&yaw_current) )
        {
            yaw_goal = yaw_remainder;
            ROS_INFO("---RADIAN ROLLOVER---");
        }

        if (yaw_current >= yaw_goal)			
        {
            currentState = HOVERING;
            movementTopic.publish(hoveringMsg);						
            if (turnToTarget == true)
            {	
                ROS_INFO("Target angle reached!");
                turnedToMove = true;				
                ros::Duration(2).sleep();
            }
        }
        else
        {
            movementTopic.publish(turnMsg_2);
        }
        yaw_prev = yaw_current;
    }

    void TurnRight_actions(tf::Quaternion quat)
    {     
        tf::Matrix3x3(quat).getRPY(roll_current, pitch_current, yaw_current);

        if (yaw_current == 0.000000)	
        {
            yaw_current = yaw_prev;
            ROS_INFO("Invalid yaw, change done.");
        }

        if ( (fabs(yaw_prev) + fabs(yaw_current) > fabs(yaw_prev + yaw_current)) && !isRadianInZeroInterval(&yaw_current) )
        {
            yaw_goal = yaw_remainder;
            ROS_INFO("---RADIAN ROLLOVER---");
        }

        if (yaw_current <= yaw_goal)
        {
            currentState = HOVERING;
            movementTopic.publish(hoveringMsg);		
            if (turnToTarget == true)
            {
                ROS_INFO("Target angle reached!");
                turnedToMove = true;
                ros::Duration(2).sleep();
            }
        }
        else
        {
            movementTopic.publish(turnMsg);
        }
        yaw_prev = yaw_current;
    }


    void MoveToTarget_actions(geometry_msgs::Point &currentPosition)
    {
        if ( isPointInsideCorridor(currentPosition, AB, BC, A, B, C) )
        {
            if ( isDroneNearTarget(&currentPosition, &StateActions::destinationCoordinate) )
            {
                movementTopic.publish(hoveringMsg);
                finish = ros::Time::now();
                currentDistance = distanceInCentimeter(&currentPosition, &StateActions::destinationCoordinate);
                double duration = finish.toSec() - start.toSec();

                currentState = TARGET_REACHED;
                ROS_INFO("TARGET REACHED (%.4f,%.4f)\n\tDistance from target: %.4f cm\n\tTravelling time: %f seconds\n", 
                    currentPosition.x, currentPosition.y, currentDistance, duration);                		
                ros::Duration(3).sleep();
            }
            else
            {
                if (helperCtr == 0)
                {
                    normalLengthOfST = distanceInMeter(&destinationCoordinate, &rotationCentre);	
                    movementCtr = 0;
                    actualDistanceFromVec = 0.0;
                    helperCtr++;
                }
                previousDistanceFromVec = actualDistanceFromVec;
                actualDistanceFromVec = getSignedDistanceFromPointToLine(rotationCentre, destinationCoordinate, currentPosition, normalLengthOfST);	
                currentDistance = distanceInMeter(&currentPosition, &StateActions::destinationCoordinate);

                adjustDroneSpeed(currentDistance, actualDistanceFromVec, previousDistanceFromVec);                
                movementTopic.publish(moveForwardMsg);

                if (movementCtr % LOGGING_MODULO == 0)
                {
                    ROS_INFO("Move to target (%f,%f)", currentPosition.x, currentPosition.y);
                    ROS_INFO("\tLinear.x: %.4f\n\tLinear.y: %.4f\n\tActual dist: %.4f\n\tPrevious dist: %.4f\n", moveForwardMsg.linear.x, moveForwardMsg.linear.y, 
                                                                    actualDistanceFromVec, previousDistanceFromVec); 
                }
                movementCtr++;
            }
        }
        else
        {
            movementTopic.publish(hoveringMsg);
            currentState = EMERGENCY_LANDING;
            ROS_FATAL("OUTSIDE THE CORRIDOR: (%.4f, %.4f)", currentPosition.x, currentPosition.y);
        }
    }

    void TargetReached_actions(bool *targetReached)
    {
        turnToTarget = false;
        turnedToMove = false;
        *targetReached = true;
    }

}

#endif