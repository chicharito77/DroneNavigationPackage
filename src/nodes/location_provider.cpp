#define NUMBER_OF_BEACONS              1
#define QUEUE_SIZE_1				1000

#include "ros/ros.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "geometry_msgs/Point.h"
#include "DroneNavigationPackage/HedgePositions.h"

using namespace std;

ros::Subscriber locationReceiver;
ros::ServiceServer serviceProvider;

std::map<int, geometry_msgs::Point> receivedLocations;


bool provide_location(DroneNavigationPackage::HedgePositions::Request &req,
                      DroneNavigationPackage::HedgePositions::Response &res  )
{
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
        }
        else
        {
            res.positions[i].address = req.addresses[i];
            res.positions[i].x_m = 0.0;
            res.positions[i].y_m = 0.0;
            res.positions[i].z_m = 0.0;
            res.positions[i].flags = 0;    
        }
    }

    return true;
}


//mivel a marvelmind koordinata rendszere balkezes volt, az y koordinata -1-gyel történő beszorzasaval
//csinaltunk belole jobbkezes koordinata-rendszert
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
            //address found
            receivedLocations[msg->address] = newPos;
        }
        else
        {
            //address not found
            receivedLocations.insert(std::pair<int, geometry_msgs::Point>( msg->address, newPos ));
        }
    }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "LocationProvider");
  ros::NodeHandle n;

  serviceProvider = n.advertiseService("/LocationProvider/provide_hedge_location", provide_location);
  locationReceiver = n.subscribe("/hedge_pos_a", QUEUE_SIZE_1, locationReceivedCallback);

  ROS_INFO("Service is ready to call ...");	
  ros::spin();

  return 0;
}