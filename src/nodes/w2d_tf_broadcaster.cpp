#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "DroneNavigationPackage/TransformParameters.h"
#include <iostream>

double X_w, Y_w, Z_w;
tf::Quaternion rotationQuaternion;
ros::Subscriber operationTopic;

void setTransformationCallback(const DroneNavigationPackage::TransformParameters::ConstPtr& msg)
{
    X_w = msg -> X_w;
    Y_w = msg -> Y_w;
    Z_w = msg -> Z_w;
    
    rotationQuaternion.setEuler(0, 0, msg->theta_dx_rad);
}

int main(int argc, char** argv)
{
    Z_w = 0.0;
    X_w = 0.0;
    Y_w = 0.0;
    rotationQuaternion.setRPY(0,0,0);

    ros::init(argc, argv, "w2d_tf_broadcaster");
    ros::NodeHandle node;
    ros::Rate r(1000);
    tf::TransformBroadcaster broadcaster;

    while(ros::ok())
    {
        operationTopic = node.subscribe("/ControlCenter/set_transform_parameters", 100, setTransformationCallback);
        broadcaster.sendTransform(
            
            tf::StampedTransform(
                            tf::Transform(rotationQuaternion, tf::Vector3(X_w, Y_w, Z_w)),
                            ros::Time::now(),"{W}", "{D}"));

                //parent //child
        ros::spinOnce();
        r.sleep();
    }
}