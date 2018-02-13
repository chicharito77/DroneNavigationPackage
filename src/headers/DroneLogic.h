#ifndef DRONELOGIC_H_INCLUDE
#define DRONELOGIC_H_INCLUDE

#define WORLD_TF_ID                    "/W"
#define ODOM_TF_ID                     "/odom"
#define BASE_LINK_TF_ID                "/ardrone_base_link"

#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string.h>
#include <stdio.h>

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
    double dcsOrientationInRad;
    double turningAngleInRad, dist, theta, x_distance, y_distance;


    geometry_msgs::Point targetCoordinate; //in DCS

    bool transformPoint(geometry_msgs::Point *destination, geometry_msgs::Point *destinationInDCS, const std::string& targetFrame, const std::string& baseFrame="/W")
    {
        tf::TransformListener listener(ros::Duration(10));

        geometry_msgs::PointStamped baseCoordinate;
        baseCoordinate.header.frame_id = baseFrame;
        baseCoordinate.header.stamp = ros::Time();
        baseCoordinate.point.x = destination->x;
        baseCoordinate.point.y = destination->y;
        baseCoordinate.point.z = destination->z;

        try{
            geometry_msgs::PointStamped transformedCoordinate;
            transformedCoordinate.point.x = 0;
            transformedCoordinate.point.y = 0;
            transformedCoordinate.point.z = 0;

            //listener.waitForTransform("/ardrone_base_link", "/W", ros::Time(0), ros::Duration(1.0) );
            //listener.transformPoint("/ardrone_base_link", baseCoordinate, transformedCoordinate);
            listener.waitForTransform(targetFrame, baseFrame, ros::Time(0), ros::Duration(1.0) );
            listener.transformPoint(targetFrame, baseCoordinate, transformedCoordinate);

            ROS_INFO("%s -> %s transform successfull:\n\t(%.2f, %.2f. %.2f) -> (%.2f, %.2f, %.2f) at time %.2f", 
                baseFrame.c_str(), targetFrame.c_str(), baseCoordinate.point.x, baseCoordinate.point.y, baseCoordinate.point.z,
                transformedCoordinate.point.x, transformedCoordinate.point.y, transformedCoordinate.point.z, transformedCoordinate.header.stamp.toSec());

            //destinationInDCS->x = base_point.point.x;
            //destinationInDCS->y = base_point.point.y;
            destinationInDCS->x = transformedCoordinate.point.y;
            destinationInDCS->y = transformedCoordinate.point.x;
            destinationInDCS->z = transformedCoordinate.point.z;
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from %s to %s:\n%s", baseFrame.c_str(), targetFrame.c_str(), ex.what());
            return false;
        }

        return true;
    }

    void calculateOrientationOfDrone(geometry_msgs::Point *offset, ros::Publisher *w2dTopic)
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
        ros::Duration(6).sleep();

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
        dcsOrientationInRad = droneOrientationInRad - degreeToRad(90.0);

        //meghívni a megfelelő topikot
        DroneNavigationPackage::TransformParameters params;
        params.X_w = offset->x;
        params.Y_w = offset->y;
        params.Z_w = 0.0;
        params.theta_dx_rad = dcsOrientationInRad;

        w2dTopic->publish(params);
        ROS_INFO("Orientation of {D} coordinate frame's y axis in {W}: %f degrees\n\n", radToDegree(droneOrientationInRad));
        ROS_INFO("w2d transform set with (%.2f, %.2f) offset and %.4f degree rotation!\n\n", offset->x, offset->y, radToDegree(dcsOrientationInRad));
        ros::Duration(2).sleep();
    }



    bool calculateDronePath(geometry_msgs::Point positionInDroneCoordinateSystem)
    {   
        //calculateDroneRouteSrvMsg.request.addresses[0] = DRONE_HEDGE_ADDRESS;

        destinationCoordinate.x = 3.2;
        destinationCoordinate.y = 0.72;
        destinationCoordinate.z = 1.0;

        //if ( positionUpdater.call(calculateDroneRouteSrvMsg) )
        {
            //droneCoordinate.x = calculateDroneRouteSrvMsg.response.positions[0].x_m;
            //droneCoordinate.y = calculateDroneRouteSrvMsg.response.positions[0].y_m;
            droneCoordinate.x = B.x;
            droneCoordinate.y = B.y;

            dist = sqrt( pow(destinationCoordinate.x - droneCoordinate.x, 2) + pow (destinationCoordinate.y - droneCoordinate.y, 2) );
            theta = atan2( (destinationCoordinate.y - droneCoordinate.y), (destinationCoordinate.x - droneCoordinate.x) );
            turningAngleInRad = theta - droneOrientationInRad;

            transformPoint(&destinationCoordinate, &targetCoordinate, BASE_LINK_TF_ID);

            ROS_INFO("\nDrone in {W}: (%.4f, %.4f)\nDestination in {W}: (%.4f, %.4f)\nDistance: %.4f m\nTheta = %f degrees\nTurning angle: %f degrees\n", 
                            droneCoordinate.x, droneCoordinate.y, destinationCoordinate.x, destinationCoordinate.y,
                            dist, radToDegree(theta), radToDegree(turningAngleInRad));


            ROS_INFO("\nDrone in {D}: (%.4f, %.4f)\nDestination in {D}: (%.4f, %.4f)\n", 
                                positionInDroneCoordinateSystem.x, positionInDroneCoordinateSystem.y, 
                                targetCoordinate.x, targetCoordinate.y);

            setMovementValues(&droneOrientationInRad, &theta, &dist, &targetCoordinate);
            return true;
        }
        
        //return false;
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
        }
        //hiba lekezelése

        return position;
    }
}


/* Your function statement here */
#endif