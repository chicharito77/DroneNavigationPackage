#define QUEUE_SIZE_1				1000
#define QUEUE_SIZE_2				 100
#define LOOP_RATE					  50

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "DroneNavigationPackage/Intervention.h"
#include "DroneNavigationPackage/TransformParameters.h"
#include "DroneNavigationPackage/HedgePositions.h"

#include "../headers/DroneLogic.h"

using namespace StateActions;

ros::Publisher takeoffTopic;
ros::Publisher landTopic;
ros::Publisher w2dTopic;
ros::Subscriber operationTopic;
ros::Subscriber externalIntervention;
ros::Subscriber setTargetTopic;
ros::Subscriber odometryTopic;

std_msgs::Empty landMsg;
std_msgs::Empty takeoffMsg;

bool running;
bool calibrationDone;
bool calculationDone;
bool targetReached;

geometry_msgs::Point positionBeforeTakeoff;
geometry_msgs::Point currentPosition;
geometry_msgs::Point currentPosInDCS;
int moveToTargetCounter = 0;


void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	DroneNavigationPackage::HedgePositions positionRefresh;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	currentPosInDCS = msg->pose.pose.position;		

	switch (currentState)
	{
		case START:
			Start_actions(quat);

		break;

//moveToTarget eltávolitva a hoveringactinonsból meg a movetotargetból
		case HOVERING:
			Hovering_actions(quat, &currentPosInDCS);

		break;

		case MOVE_TO_TARGET:
			MoveToTarget_actions(&currentPosInDCS);
			
		break;

		case TARGET_REACHED:
			TargetReached_actions(&targetReached);

		break;

		case TURN_RIGHT:		//negatív radián érték mindvégig
			TurnRight_actions(quat, &DroneLogic::inTurningStateCounter);
		
		break;

		case TURN_LEFT:
			TurnLeft_actions(quat, &DroneLogic::inTurningStateCounter);

		break;

	}
}

void setTargetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	destinationCoordinate.x = msg -> x;
	destinationCoordinate.y = msg -> y;
	destinationCoordinate.z = msg -> z;

	ROS_INFO("New target in {W} saved! (%f, %f, %f)", destinationCoordinate.x, destinationCoordinate.y, destinationCoordinate.z);
	calculationDone = false;
	targetReached = false;
}

void interventionCallback(const DroneNavigationPackage::Intervention::ConstPtr& msg)
{
	if (running == false && msg->READY_TO_START == true)
	{
		positionBeforeTakeoff = DroneLogic::getPositionOfDrone();
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
		running = false;
		landTopic.publish( landMsg );
		ROS_FATAL("Drone landing requested due to emergency!");
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
			case 1:		//calibration
				DroneLogic::calculateOrientationOfDrone(&positionBeforeTakeoff, &w2dTopic);
				calibrationDone = true;
				calculationDone = false;
			break;
	
			case 2:
				if(calibrationDone)
				{
					calculationDone = DroneLogic::calculateDronePath(currentPosInDCS);
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
					ROS_INFO("Reaching target requires calibration and path calculations! At least one of them missing!");
				}
	
			break;

			/*case 4:
				movementTopic.publish( liftingMsg );
				ros::Duration(3).sleep();
				movementTopic.publish( hoveringMsg );
			break;*/

		}
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ControlCenter");
	ros::NodeHandle n;
	
	//if(ros::service::waitForService("/ardrone/flattrim", ros::Duration(10)) )
	takeoffTopic = n.advertise<std_msgs::Empty>("/ardrone/takeoff", QUEUE_SIZE_1);
	movementTopic = n.advertise<geometry_msgs::Twist>("/cmd_vel", QUEUE_SIZE_1);	
	landTopic = n.advertise<std_msgs::Empty>("/ardrone/land", QUEUE_SIZE_1);
	w2dTopic = n.advertise<DroneNavigationPackage::TransformParameters>("/ControlCenter/set_transform_parameters", QUEUE_SIZE_2);	
	
	operationTopic = n.subscribe("/ControlCenter/drone_operations", QUEUE_SIZE_2, droneOperationCallback);
	externalIntervention = n.subscribe("/ControlCenter/external_intervention", QUEUE_SIZE_2, interventionCallback);
	setTargetTopic = n.subscribe("/ControlCenter/set_target", QUEUE_SIZE_2, setTargetCallback);
	odometryTopic = n.subscribe("/ardrone/odometry", QUEUE_SIZE_1, odometryCallback);

	positionUpdater = n.serviceClient<DroneNavigationPackage::HedgePositions>("/LocationProvider/provide_hedge_location");
	ros::Rate loop_rate(LOOP_RATE);	

	system("rosservice call /ardrone/flattrim");
	ros::Duration(1).sleep();	
	ROS_INFO("Node started ...");	
	
	initializeMovementMessages();
	
	running = false;
	currentState = START;
	ros::spin();
	
	return 0;
}