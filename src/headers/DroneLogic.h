#ifndef DRONELOGIC_H_INCLUDE
#define DRONELOGIC_H_INCLUDE

#include "ros/ros.h"

#include "StateMachine.h"

using namespace StateActions;

namespace DroneLogic
{
    DroneNavigationPackage::HedgePositions coordinateSystemSynchSrvMsg, calculateDroneRouteSrvMsg;
    geometry_msgs::Point A, B;                                                                  //variables for the calculation phase
    geometry_msgs::Point droneCoordinate, destinationCoordinate;

    int numberOfMeasurements = 0;
    int inTurningStateCounter = 0;

    double droneOrientationInRad;
    double turningAngleInRad, dist, theta, x_distance, y_distance;

    double droneOrientationByLocalData;
    geometry_msgs::Point destinationCoordinateInDroneCoordinateSystem;


    void calculateOrientationOfDrone()
    {
        int inCalculationStateCounter = 0;
        coordinateSystemSynchSrvMsg.request.addresses[0] = DRONE_HEDGE_ADDRESS;
        
        while (!positionUpdater.call(coordinateSystemSynchSrvMsg))
        {
            if (inCalculationStateCounter == 0)
            {
                ROS_WARN("Location update service unavailable! Waiting for location...");
                inCalculationStateCounter++;
            }
        }
        A.x = coordinateSystemSynchSrvMsg.response.positions[0].x_m;
        A.y = coordinateSystemSynchSrvMsg.response.positions[0].y_m;
        ROS_INFO("\tPoint A measured! (%f, %f)", A.x, A.y);
        coordinateSystemSynchSrvMsg.response.positions[0].x_m = 0.0;
        coordinateSystemSynchSrvMsg.response.positions[0].y_m = 0.0;			

        movementTopic.publish(calibrationMsg);
        ros::Duration(1).sleep();
        movementTopic.publish(hoveringMsg);
        ros::Duration(4).sleep();

        inCalculationStateCounter = 0;
        while (!positionUpdater.call(coordinateSystemSynchSrvMsg))
        {
            if (inCalculationStateCounter == 0)
            {
                ROS_WARN("Location update service unavailable! Waiting for location...");
                inCalculationStateCounter++;
            }
        }
        B.x = coordinateSystemSynchSrvMsg.response.positions[0].x_m;
        B.y = coordinateSystemSynchSrvMsg.response.positions[0].y_m;
        ROS_INFO("\tPoint B measured! (%f, %f)", B.x, B.y);	
        
        droneOrientationInRad = atan2( (A.y - B.y), (A.x - B.x) );
        ROS_INFO("Orientation of {D} coordinate frame's y axis in {W}: %f degrees\n\n", radToDegree(droneOrientationInRad));
        ros::Duration(2).sleep();
    }



    bool calculateDronePath(geometry_msgs::Point positionInDroneCoordinateSystem)
    {   
        calculateDroneRouteSrvMsg.request.addresses[0] = DRONE_HEDGE_ADDRESS;

        destinationCoordinate.x = 4.8;
        destinationCoordinate.y = -1.755;
        destinationCoordinate.z = 1.0;

        destinationCoordinateInDroneCoordinateSystem.x = 0.0;
        destinationCoordinateInDroneCoordinateSystem.y = 0.0;
        destinationCoordinateInDroneCoordinateSystem.z = 0.0;

        if ( positionUpdater.call(calculateDroneRouteSrvMsg) )
        {
            droneCoordinate.x = calculateDroneRouteSrvMsg.response.positions[0].x_m;
            droneCoordinate.y = calculateDroneRouteSrvMsg.response.positions[0].y_m;

            dist = sqrt( pow(destinationCoordinate.x - droneCoordinate.x, 2) + pow (destinationCoordinate.y - droneCoordinate.y, 2) );
            theta = atan2( (destinationCoordinate.y - droneCoordinate.y), (destinationCoordinate.x - droneCoordinate.x) );
            turningAngleInRad = theta - droneOrientationInRad;

            destinationCoordinateInDroneCoordinateSystem.x = positionInDroneCoordinateSystem.x;
            destinationCoordinateInDroneCoordinateSystem.y = (positionInDroneCoordinateSystem.y + dist);
            destinationCoordinateInDroneCoordinateSystem.z = positionInDroneCoordinateSystem.z;

            ROS_INFO("\nDrone in {W}: (%.4f, %.4f)\nDestination in {W}: (%.4f, %.4f)\nDistance: %.4f m\nTheta = %f degrees\nTurning angle: %f degrees\n", 
                            droneCoordinate.x, droneCoordinate.y, destinationCoordinate.x, destinationCoordinate.y,
                            dist, radToDegree(theta), radToDegree(turningAngleInRad));

            ROS_INFO("\nDrone in {D}: (%.4f, %.4f)\nDestination in {D}: (%.4f, %.4f)\n", 
                                positionInDroneCoordinateSystem.x, positionInDroneCoordinateSystem.y, 
                                destinationCoordinateInDroneCoordinateSystem.x, destinationCoordinateInDroneCoordinateSystem.y);       

            //setMovementValues(&droneOrientationInRad, &theta, &dist, &destinationCoordinateInDroneCoordinateSystem);
            setMovementValues(&droneOrientationInRad, &theta, &dist, &destinationCoordinate);
            return true;
        }
        
        return false;
    }

    

    void goToTarget()
    {
        turnToTarget = true;
        yaw_goal = yaw_prev = yaw_remainder = 0.0;
        inTurningStateCounter = 0;
        ROS_INFO("starting yaw in rad: %f", yaw_starting);

        yaw_goal = yaw_starting + turningAngleInRad;
        yaw_prev = yaw_starting;

        if (turningAngleInRad >= 0)				//must turn left
        {
            currentState = TURN_LEFT;
            ROS_INFO("%d. goal := %f", numberOfMeasurements++, yaw_goal);

            if (yaw_goal > M_PI)
            {
                yaw_remainder = -M_PI + (yaw_goal + -M_PI);
                ROS_INFO("yaw_remainder = %f", yaw_remainder);
            }

            movementTopic.publish(turnMsg_2);
        }
        else								//must turn right
        {
            currentState = TURN_RIGHT;									//dummy value 
            ROS_INFO("%d. goal := %f", numberOfMeasurements++, yaw_goal);

            if (yaw_goal < -M_PI)
            {
                yaw_remainder = M_PI + (yaw_goal + M_PI);
                ROS_INFO("yaw_remainder = %f", yaw_remainder);
            }	

            movementTopic.publish(turnMsg);
        }
    }

    geometry_msgs::Point getPositionOfDrone()
    {
        geometry_msgs::Point position;
        coordinateSystemSynchSrvMsg.request.addresses[0] = DRONE_HEDGE_ADDRESS;
        
        if (positionUpdater.call(coordinateSystemSynchSrvMsg))
        {
            position.x = coordinateSystemSynchSrvMsg.response.positions[0].x_m;
            position.y = coordinateSystemSynchSrvMsg.response.positions[0].y_m;
            position.z = 1.0;

            droneCoordinate = position;
        }
        //hiba lekezelése
    }
}


/* Your function statement here */
#endif