#ifndef STATEMACHINE_H_INCLUDE
#define STATEMACHINE_H_INCLUDE

#define START								0x00
#define HOVERING							0x01
#define TURN_RIGHT							0x02
#define TURN_LEFT							0x03
#define MOVE_TO_TARGET						0x04
#define TARGET_REACHED						0x05

#define DRONE_HEDGE_ADDRESS					  12

#define MODULO                               200

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"

#include "DroneNavigationPackage/HedgePositions.h"
#include "CoordinateGeometryCalculations.h"


namespace StateActions
{
    geometry_msgs::Twist  turnMsg, turnMsg_2, hoveringMsg, calibrationMsg, moveForwardMsg, liftingMsg;
    DroneNavigationPackage::HedgePositions positionRefresh;

    ros::Publisher movementTopic;

    ros::ServiceClient positionUpdater;
 
    double roll_starting, pitch_starting, yaw_starting, roll_current, pitch_current, yaw_current;
    double yaw_prev, yaw_goal, yaw_remainder;
    
    int currentState;
    bool turnToTarget = false;
    bool turnedToMove = false;
    int invalidLocationUpdateCounter = 0;
    geometry_msgs::Point destinationCoordinate;

    bool inTestState = false;
    double ThetaZero, Theta, Distance;

    void setMovementValues(double *theta_zero, double *theta, double *distance, geometry_msgs::Point *_destinationCoordinate)
    {
        ThetaZero = *theta_zero;
        Theta = *theta;
        Distance = *distance;
        destinationCoordinate.x = _destinationCoordinate->x;
        destinationCoordinate.y = _destinationCoordinate->y;
        destinationCoordinate.z = _destinationCoordinate->z;
    }


    void initializeMovementMessages()
    {
        hoveringMsg.linear.x = 0.0;
        hoveringMsg.linear.y = 0.0;
        hoveringMsg.linear.z = 0.0;
        hoveringMsg.angular.x = 0.0;
        hoveringMsg.angular.y = 0.0;
        hoveringMsg.angular.z = 0.0;
        liftingMsg.linear.z = 0.2;

        turnMsg.angular.z = -0.15;
        turnMsg_2.angular.z = 0.15;
        calibrationMsg.linear.x = -0.1;    
        moveForwardMsg.linear.x = 0.08;
    }


    void Start_actions(tf::Quaternion quat)
    {
        tf::Matrix3x3(quat).getRPY(roll_starting,pitch_starting,yaw_starting);
        movementTopic.publish(hoveringMsg);
    }

    void Hovering_actions(tf::Quaternion quat, geometry_msgs::Point *currentPosInDCS, int *moveToTargetCtr)
    {
        tf::Matrix3x3(quat).getRPY(roll_starting,pitch_starting,yaw_starting);
        movementTopic.publish(hoveringMsg);

        if (turnToTarget == true && turnedToMove == true)
        {
            *moveToTargetCtr = 0;
            ros::Duration(2).sleep();
            currentState = MOVE_TO_TARGET;
            movementTopic.publish(moveForwardMsg);
            ROS_INFO("travelling to target ...");
        }
    }

    void TurnLeft_actions(tf::Quaternion quat, int *counter, geometry_msgs::Point *currentPos)
    {
        //yaw_prev = (*counter == 0) ? yaw_starting : yaw_current;
        //*counter++;	        
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
            ROS_INFO("\tcurrent: %f", yaw_current);
        }

        yaw_prev = yaw_current;
    }
    
    void TurnRight_actions(tf::Quaternion quat, int *counter, geometry_msgs::Point *currentPos)
    {
        //yaw_prev = (*counter == 0) ? yaw_starting : yaw_current;
        //*counter++;        
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
            ROS_INFO("\tcurrent: %f", yaw_current);
        }

        yaw_prev = yaw_current;
    }


    void MoveToTarget_actions(geometry_msgs::Point *currentPosition, int *moveToTargetCtr)
    {
        if ( isDroneInRadius(currentPosition, &StateActions::destinationCoordinate) )
        {
            movementTopic.publish(hoveringMsg);
            ros::Duration(1).sleep();
            currentState = TARGET_REACHED;
            ROS_INFO("TARGET REACHED (%f,%f)->(%f,%f)", currentPosition->x, currentPosition->y, StateActions::destinationCoordinate.x, StateActions::destinationCoordinate.y);
        }
        else
        {
            movementTopic.publish(moveForwardMsg);
            if (*moveToTargetCtr % MODULO == 0)
            {
                ROS_INFO("move to target (%f,%f)->(%f,%f)", currentPosition->x, currentPosition->y, StateActions::destinationCoordinate.x, StateActions::destinationCoordinate.y);
            }
            *moveToTargetCtr++;
            
        }
    }

    void TargetReached_actions(bool *targetReached)
    {
        turnToTarget = false;
        turnedToMove = false;
        *targetReached = true;
        currentState = HOVERING;
    }
}

/* Your function statement here */
#endif