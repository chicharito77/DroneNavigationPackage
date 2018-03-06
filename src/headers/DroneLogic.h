#ifndef DRONELOGIC_H_INCLUDE
#define DRONELOGIC_H_INCLUDE

#define WORLD_TF_ID                    "/W"
#define ODOM_TF_ID                     "/odom"
#define BASE_LINK_TF_ID                "/ardrone_base_link"
#define BASE_FRONTCAM_TF_ID            "/ardrone_base_frontcam"
#define BASE_BOTTOMCAM_TF_ID           "/ardrone_base_bottomcam"

#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string.h>
#include <stdio.h>

#include "StateMachine.h"

using namespace StateActions;

namespace DroneLogic
{
    DroneNavigationPackage::HedgePositions marvelmindPositionRequestMsg;
    geometry_msgs::Point A, B;                                              //variables for the calculation phase
    geometry_msgs::Point droneCoordinate, destinationCoordinate;
    
    //MAP data
    geometry_msgs::Point startCoordinate, targetCoordinate;                 //in DCS
    geometry_msgs::Point S_A, S_B, D_C, D_D;
    geometry_msgs::Vector3 AB, BC;

    int serverRequestCounter;
    double droneOrientationInRad;
    double dcsOrientationInRad;
    double turningAngleInRad, dist, theta;
    double dist_2;                          //calculated distance in droneCoordSystem



    void getPositionOfDrone(geometry_msgs::Point *position)
    {
        marvelmindPositionRequestMsg.request.addresses[0] = DRONE_HEDGE_ADDRESS;
        marvelmindPositionRequestMsg.response.positions[0].x_m = 0.0;
        marvelmindPositionRequestMsg.response.positions[0].y_m = 0.0;
        marvelmindPositionRequestMsg.response.positions[0].z_m = 0.0;        
        serverRequestCounter = 0;

        while (!positionUpdater.call(marvelmindPositionRequestMsg))
        {
            if (serverRequestCounter == 0)
            {
                ROS_WARN("Location update service unavailable! Waiting for location...");
                serverRequestCounter++;
            }
        }

        position->x = marvelmindPositionRequestMsg.response.positions[0].x_m;
        position->y = marvelmindPositionRequestMsg.response.positions[0].y_m;
        position->z = 0.0;
    }

    
    void createCorridor()
    {
        double R_diagonal = sqrt( pow(dist_2,2) + pow(EPSILON_RADIUS_FROM_TARGET, 2) );

        //bisection method
        geometry_msgs::Point helper;
        double R_orig = R_diagonal;
        double R_increment;
        double tangentOfDS;
        double ratio;
        double lengthOfAB;
        int iterationStep = 0;
        
        tangentOfDS= atan2( startCoordinate.y - targetCoordinate.y, startCoordinate.x - targetCoordinate.x );
        helper.x = startCoordinate.x + EPSILON_RADIUS_FROM_TARGET*cos(tangentOfDS);
        helper.y = startCoordinate.y + EPSILON_RADIUS_FROM_TARGET*sin(tangentOfDS);
        R_increment = EPSILON_RADIUS_FROM_TARGET / 2.0;

        do
        {
            iterationStep++;            
            intersectionPointsOfCircles(startCoordinate, EPSILON_RADIUS_FROM_TARGET,
                                    targetCoordinate, R_diagonal, &S_A, &S_B);

            lengthOfAB = distanceFromTargetInMeter(&S_A, &S_B);
            ratio = EPSILON_RADIUS_FROM_TARGET / lengthOfAB;
            
            if (  !(ratio >= 0.8 && ratio <= 0.9) )
            {
                if (ratio < 0.8)
                {
                    R_diagonal = R_diagonal + (R_increment);
                }
                else
                {
                    R_diagonal = R_diagonal - (R_increment);
                }
                R_increment /= 2.0;
            }  
        }
        while( !(ratio >= 0.8 && ratio <= 0.9) );

        ROS_INFO("R_diag found in %d. iteration (%.4f -> %.4f)", iterationStep, R_orig, R_diagonal);

        /*intersectionPointsOfCircles(startCoordinate, EPSILON_RADIUS_FROM_TARGET,
                                    targetCoordinate, R_diagonal, &S_A, &S_B);*/
        intersectionPointsOfCircles(startCoordinate, R_diagonal,
                                    targetCoordinate, EPSILON_RADIUS_FROM_TARGET, &D_C, &D_D);

        //BC Must be perpendicular to AB
        AB = createVector(S_A, S_B);
        getPerpendicularVector(AB, S_B, BC, D_C, D_D);

        //listázni aszámolás eredményeit, majd ezeket átmásolni statemachine-ba!!!
        ROS_INFO("Travel corridor calculated!\n\tA(%.2f; %.2f)\tB(%.2f; %.2f)\n\tC(%.2f; %.2f)\tD(%.2f; %.2f)\n\tAB vector->(%.4f; %.4f)\n\tBC vector->(%.4f; %.4f)\n", 
                    S_A.x, S_A.y, S_B.x, S_B.y, D_C.x, D_C.y, D_D.x, D_D.y,
                    AB.x, AB.y, BC.x, BC.y);
    }


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

            listener.waitForTransform(targetFrame, baseFrame, ros::Time(0), ros::Duration(1.0) );
            listener.transformPoint(targetFrame, baseCoordinate, transformedCoordinate);
            ROS_INFO("%s -> %s transform successfull!\n", baseFrame.c_str(), targetFrame.c_str());

            /*ROS_INFO("%s -> %s transform successfull!\n\t(%.2f, %.2f. %.2f) -> (%.2f, %.2f, %.2f) at time %.2f\n", 
                baseFrame.c_str(), targetFrame.c_str(), baseCoordinate.point.x, baseCoordinate.point.y, baseCoordinate.point.z,
                transformedCoordinate.point.x, transformedCoordinate.point.y, transformedCoordinate.point.z, transformedCoordinate.header.stamp.toSec());*/

            destinationInDCS->x = transformedCoordinate.point.x;
            destinationInDCS->y = transformedCoordinate.point.y;
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
        getPositionOfDrone(&A);
        ROS_INFO("\tPoint A measured! (%f, %f)", A.x, A.y);

        movementTopic.publish(calibrationMsg);
        ros::Duration(1).sleep();
        movementTopic.publish(hoveringMsg);
        ros::Duration(6).sleep();

        getPositionOfDrone(&B);
        ROS_INFO("\tPoint B measured! (%f, %f)", B.x, B.y);	        
        droneOrientationInRad = atan2( (A.y - B.y), (A.x - B.x) );
        dcsOrientationInRad = droneOrientationInRad - degreeToRad(90.0);

        DroneNavigationPackage::TransformParameters params;
        params.X_w = offset->x;
        params.Y_w = offset->y;
        params.Z_w = 0.0;
        params.theta_dx_rad = dcsOrientationInRad;

        w2dTopic->publish(params);
        ROS_INFO("Orientation of {D} coordinate frame's y axis in {W}: %f degrees", radToDegree(droneOrientationInRad));
        ROS_INFO("W2D transform set with (%.2f, %.2f) offset and %.4f degree rotation!\n\n", offset->x, offset->y, radToDegree(dcsOrientationInRad));
    }



    bool calculateDronePath(geometry_msgs::Point positionInDroneCoordinateSystem)
    {   
        bool result;
        destinationCoordinate.x = 3.218;
        destinationCoordinate.y = 0.65;
        destinationCoordinate.z = 1.0;

        droneCoordinate.x = B.x;
        droneCoordinate.y = B.y;

        dist = sqrt( pow(destinationCoordinate.x - droneCoordinate.x, 2) + pow (destinationCoordinate.y - droneCoordinate.y, 2) );
        theta = atan2( (destinationCoordinate.y - droneCoordinate.y), (destinationCoordinate.x - droneCoordinate.x) );
        turningAngleInRad = theta - droneOrientationInRad;

        //don't turn bigger angle than abs(M_PI)
        if (fabs(turningAngleInRad) > M_PI)
        {
            turningAngleInRad = 2*M_PI + (turningAngleInRad < 0 ? 1 : -1)*turningAngleInRad;
            ROS_INFO("recalc done\n");
        }

        result = transformPoint(&destinationCoordinate, &targetCoordinate, BASE_BOTTOMCAM_TF_ID);
        startCoordinate = positionInDroneCoordinateSystem;

        dist_2 = sqrt( pow(targetCoordinate.x - startCoordinate.x, 2) + pow (targetCoordinate.y - startCoordinate.y, 2) );

        if (result)
        {
            ROS_INFO("\n\tDrone in {W}: (%.4f, %.4f)\n\tDestination in {W}: (%.4f, %.4f)\n\tDistance: %.4f m\n\tTheta = %f degrees\n\tTurning angle: %f degrees\n", 
                            droneCoordinate.x, droneCoordinate.y, destinationCoordinate.x, destinationCoordinate.y,
                            dist, radToDegree(theta), radToDegree(turningAngleInRad));

            ROS_INFO("\n\tDrone in {D}: (%.4f, %.4f)\n\tDestination in {D}: (%.4f, %.4f)\n\tDistance: %.4f m\n", 
                                positionInDroneCoordinateSystem.x, positionInDroneCoordinateSystem.y, 
                                targetCoordinate.x, targetCoordinate.y, dist_2);

            createCorridor();

            setMovementValues(&droneOrientationInRad, &theta, &dist, &targetCoordinate, &startCoordinate,
                                &S_A, &S_B, &D_C, &AB, &BC);
            //adjustDroneSpeed(dist);
            adjustDroneSpeed(dist_2);
        }        

        return result;
    }

    

    void goToTarget()
    {
        turnToTarget = true;
        yaw_goal = yaw_prev = yaw_remainder = 0.0;
        ROS_INFO("starting yaw in rad: %f", yaw_starting);

        yaw_goal = yaw_starting + turningAngleInRad;
        yaw_prev = yaw_starting;

        if (turningAngleInRad >= 0)				//must turn left
        {
            currentState = TURN_LEFT;
            ROS_INFO("Goal := %f", yaw_goal);

            if (yaw_goal > M_PI)
            {
                yaw_remainder = -M_PI + (yaw_goal + -M_PI);
                ROS_INFO("yaw_remainder = %f", yaw_remainder);
            }

            movementTopic.publish(turnMsg_2);
        }
        else								//must turn right
        {
            currentState = TURN_RIGHT;
            ROS_INFO("Goal := %f", yaw_goal);

            if (yaw_goal < -M_PI)
            {
                yaw_remainder = M_PI + (yaw_goal + M_PI);
                ROS_INFO("yaw_remainder = %f", yaw_remainder);
            }	

            movementTopic.publish(turnMsg);
        }
    }

}


/* Your function statement here */
#endif