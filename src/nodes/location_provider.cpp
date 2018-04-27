#define NUMBER_OF_BEACONS                                           1
#define QUEUE_SIZE_1				                             1000
#define STARTUP_TIMEOUT                                            10

#define MM_TOPICNAME                                    "/hedge_pos_a"
#define SERVICENAME         "/LocationProvider/provide_hedge_location"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "ros/ros.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "geometry_msgs/Point.h"
#include "ros/master.h"
#include "DroneNavigationPackage/HedgePositions.h"

using namespace std;

ros::Subscriber locationReceiver;
ros::ServiceServer serviceProvider;

std::map<int, geometry_msgs::Point> receivedLocations;
int requestCounter;


bool isTopicAvailable(const string& topic)
{
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++) 
    {
        const ros::master::TopicInfo& info = *it;
        if(info.name.compare(topic)==0)
        {
            return true;
        }
    }
    return false;
}


bool provide_location(DroneNavigationPackage::HedgePositions::Request &req,
                      DroneNavigationPackage::HedgePositions::Response &res  )
{
    requestCounter++;
    int i=0;
    map<int, geometry_msgs::Point>::iterator it;

    for(; i<NUMBER_OF_BEACONS; i++)
    {
        it = receivedLocations.find(req.addresses[i]);
        if (it != receivedLocations.end())
        {
            geometry_msgs::Point pi = receivedLocations[req.addresses[i]];
            res.positions[i].address = req.addresses[i];
            res.positions[i].x_m = pi.x;
            res.positions[i].y_m = pi.y;       
            res.positions[i].z_m = pi.z;
            res.positions[i].flags = 1;
            ROS_INFO("\t%d. position request for hedge_%d was successful!\n", requestCounter, req.addresses[i]);
        }
        else
        {
            res.positions[i].address = req.addresses[i];
            res.positions[i].x_m = 0.0;
            res.positions[i].y_m = 0.0;
            res.positions[i].z_m = 0.0;
            res.positions[i].flags = 0;  
            ROS_INFO("\t%d. position request failed: hedge_%d unknown!\n", requestCounter, req.addresses[i]);  
        }
    }

    return true;
}


void locationReceivedCallback(const marvelmind_nav::hedge_pos_a::ConstPtr& msg)
{
    if ( msg->flags == 2 )
    {
        map<int, geometry_msgs::Point>::iterator it = receivedLocations.find(msg->address);
        geometry_msgs::Point newPos;
        newPos.x = msg->x_m;
        newPos.y = msg->y_m;
        newPos.z = msg->z_m;
        
        if (it != receivedLocations.end())
        {
            receivedLocations[msg->address] = newPos;
        }
        else
        {
            receivedLocations.insert(std::pair<int, geometry_msgs::Point>( msg->address, newPos ));
        }
    }
    else
    {
        ROS_INFO("***Not valid data received, storing process cancelled!\n");
    }
}


void startNode(ros::NodeHandle* node)
{
    time_t start, current;
    start = time(NULL);
    int counter = 0;
    requestCounter = 0;
    
    do
    {
        if (isTopicAvailable(MM_TOPICNAME))
        {
            serviceProvider = node->advertiseService(SERVICENAME, provide_location);
            locationReceiver = node->subscribe(MM_TOPICNAME, QUEUE_SIZE_1, locationReceivedCallback);
            ROS_INFO("Service is ready to call ...\n");
            ros::spin();
            break;
        }
        else
        {
            if (counter==0)
            {
                ROS_INFO("Waiting 30 seconds for '%s' topic to be available ...\n", MM_TOPICNAME);
                counter++;
            }
        }
        current = time(NULL);

    }while( difftime(current, start) < STARTUP_TIMEOUT );
    ROS_ERROR("TIMEOUT: Node is not starting because dependency topic does not exist!\n");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "LocationProvider");
    ros::NodeHandle n;

    startNode(&n);

    return 0;
}