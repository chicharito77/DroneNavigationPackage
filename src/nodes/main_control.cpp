#define QUEUE_SIZE_1							  						1000
#define QUEUE_SIZE_2				 			   						 100
#define LOOP_RATE					 			   						 150

#define TAKEOFF_TOPICNAME								   "/ardrone/takeoff"
#define MOVEMENT_TOPICNAME										   "/cmd_vel"	
#define LANDING_TOPICNAME				   					  "/ardrone/land"
#define W2D_TOPICNAME				"/ControlCenter/set_transform_parameters"
#define DRONEOPS_TOPICNAME				    "/ControlCenter/drone_operations"
#define INTERVENTION_TOPICNAME		   "/ControlCenter/external_intervention"
#define NEWTARGET_TOPICNAME						  "/ControlCenter/set_target"
#define ODOMETRY_TOPICNAME 								  "/ardrone/odometry"
#define POSITION_SERVICENAME	   "/LocationProvider/provide_hedge_location"

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "DroneNavigationPackage/Intervention.h"
#include "DroneNavigationPackage/TransformParameters.h"
#include "DroneNavigationPackage/HedgePositions.h"

#include "../headers/DroneLogic.h"

ros::Publisher takeoffTopic;
ros::Publisher landTopic;
ros::Publisher w2dTopic;
ros::Subscriber operationTopic;
ros::Subscriber externalInterventionTopic;
ros::Subscriber setTargetTopic;
ros::Subscriber odometryTopic;

std_msgs::Empty landMsg;
std_msgs::Empty takeoffMsg;

bool running;
bool calibrationDone;
bool calculationDone;
bool targetReached;

geometry_msgs::Point positionBeforeTakeoff;		//in {W}
geometry_msgs::Point positionInBottomcamFrame;	//in DCS (droneCoordinateSystem)


void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);		

	switch (currentState)
	{
		case START:
			Start_actions(quat);
		break;

		case HOVERING:
			if (calibrationDone)
			{
				positionInBottomcamFrame.x = 0.0;
				positionInBottomcamFrame.y = 0.0;
				positionInBottomcamFrame.z = 0.0;
				positionInBottomcamFrame = msg->pose.pose.position;
			}			

			Hovering_actions(quat);
		break;

		case MOVE_TO_TARGET:
			positionInBottomcamFrame.x = 0.0;
			positionInBottomcamFrame.y = 0.0;
			positionInBottomcamFrame.z = 0.0;
			positionInBottomcamFrame = msg->pose.pose.position;

			MoveToTarget_actions(positionInBottomcamFrame);
		break;

		case TARGET_REACHED:
			TargetReached_actions(&targetReached);
			landTopic.publish( landMsg );
			ROS_INFO("Drone landed successfully!\n");

			ros::shutdown();
		break;

		case TURN_RIGHT:
			TurnRight_actions(quat);		
		break;

		case TURN_LEFT:
			TurnLeft_actions(quat);
		break;

		case EMERGENCY_LANDING:
			landTopic.publish( landMsg );
			running = false;
			ROS_FATAL("Drone landing requested due to stepping over the authorized area!");			
			ros::shutdown();
		break;
	}
}


void setTargetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	destinationCoordinate.x = msg -> x;
	destinationCoordinate.y = msg -> y;
	destinationCoordinate.z = 0.0;			//this system works in 2D space

	ROS_INFO("New target in {W} saved: (%f, %f)\n", destinationCoordinate.x, destinationCoordinate.y);
	calculationDone = false;
	targetReached = false;
}


void interventionCallback(const DroneNavigationPackage::Intervention::ConstPtr& msg)
{
	if (running == false && msg->READY_TO_START == true)
	{
		DroneLogic::getPositionOfDrone(&positionBeforeTakeoff);
        ros::Duration(3).sleep();
		ROS_INFO("Initialization point in {W}: (%.2f, %.2f)\n", positionBeforeTakeoff.x, positionBeforeTakeoff.y);
		takeoffTopic.publish( takeoffMsg );

		running = true;
		calibrationDone = false;
		calculationDone = false;
		targetReached = false;
		currentState = HOVERING;
	}

	if (running == true && msg->EMERGENCY_EXIT_HAPPENED == true)
	{
		landTopic.publish( landMsg );
		ROS_FATAL("Drone landing requested due to emergency!");

		running = false;
		ros::shutdown();
	}

	if (running == true && msg->LAND_DRONE == true)
	{
		landTopic.publish( landMsg );
		ROS_INFO("Drone landed. Ready to use again!\n");
		
		running = false;
		calibrationDone = false;
		calculationDone = false;
		targetReached = false;
		currentState = START;
	}
}


void droneOperationCallback(const std_msgs::Int8::ConstPtr& msg)
{
	if (running)
	{
		switch (msg->data)
		{
			case 1:
				DroneLogic::calculateOrientationOfDrone(&positionBeforeTakeoff, &w2dTopic);
				calibrationDone = true;
				calculationDone = false;
			break;
	
			case 2:
				if(calibrationDone)
				{
					calculationDone = DroneLogic::calculateDronePath(positionInBottomcamFrame);
					targetReached = false;
										
					if (!calculationDone)
					{
						ROS_INFO("Path calculation failed! Execute path calculation again!");
					}
				}
				else
				{
					ROS_INFO("Path calculation requires calibration first! Send this type of message with value 1!");
				}
			break;
	
			case 3:
				if (calibrationDone && calculationDone)
				{
					DroneLogic::goToTarget();
				}
				else
				{
					ROS_INFO("Reaching target requires calibration and path calculations! At least one of them is missing!");
				}
			break;
		}
	}
	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ControlCenter");
	ros::NodeHandle mainNode;

	takeoffTopic = mainNode.advertise<std_msgs::Empty>(TAKEOFF_TOPICNAME, QUEUE_SIZE_1);
	movementTopic = mainNode.advertise<geometry_msgs::Twist>(MOVEMENT_TOPICNAME, QUEUE_SIZE_1);	
	landTopic = mainNode.advertise<std_msgs::Empty>(LANDING_TOPICNAME, QUEUE_SIZE_1);
	w2dTopic = mainNode.advertise<DroneNavigationPackage::TransformParameters>(W2D_TOPICNAME, QUEUE_SIZE_2);	
	
	operationTopic = mainNode.subscribe(DRONEOPS_TOPICNAME, QUEUE_SIZE_2, droneOperationCallback);
	externalInterventionTopic = mainNode.subscribe(INTERVENTION_TOPICNAME, QUEUE_SIZE_2, interventionCallback);
	setTargetTopic = mainNode.subscribe(NEWTARGET_TOPICNAME, QUEUE_SIZE_2, setTargetCallback);
	odometryTopic = mainNode.subscribe(ODOMETRY_TOPICNAME, QUEUE_SIZE_1, odometryCallback);

	positionUpdater = mainNode.serviceClient<DroneNavigationPackage::HedgePositions>(POSITION_SERVICENAME);

	ros::Rate loop_rate(LOOP_RATE);	
	system("rosservice call /ardrone/flattrim");
	ROS_INFO("ControlCenter started ...");		
	initializeMovementMessages();
	
	running = false;
	currentState = START;
	ros::spin();
	
	return 0;
}