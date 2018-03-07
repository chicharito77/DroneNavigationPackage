This is a thesis project (title: Robot control based on ROS) at University of Pannonia.
Used technologies:
    - Oracle VM VirtualBox (Windows 10 host)
    - Ubuntu 14.04 guest OS
    - ROS Indigo Igloo
    - Marvelmind indoor localization system
    - Parrot ARDrone 2.0
    
The goal of my thesis work is to control an ArDrone 2.0 with the help of an ultrasonic indoor localization system. 
This navigation system is developed by Marvelmind Robotics, and the localization and navigation of the drone is based on it. 
There is a „moving” ultrasonic beacon placed on the hull of the drone. In the testing environment there are at least 3 stationary beacons
on the wall or ceiling. Those devices forms a map: the environment is measured by ultrasonic signals. 
The drone moves in a 2D coordinate system (height of the milestones is ignored). 

If you have any suggestions and remark, please contact me.

v1.0: The drone can travel from a starting position to a constant destination position (located in DroneLogic.h file's calculateDronePath
method). The achieving of the destination position is supervised in the drone's coordinate system.

Required ROS packages:
  	- ardrone_autonomy
  	- marvelmind_nav
  	
Usage of package: the order of node starting is
  0. roscore
  1. marvelmind_nav package's hedge_rcv_bin node
  2. ardrone_autonomy node, with these parameters
        rosrun ardrone_autonomy ardrone_driver _navdata_demo:=1 _realtime_navdata:=True _looprate:=15 _enable_legacy_navdata:=False
  3. DroneNavigationPackage's LocationProvider node
  4. DroneNavigationPackage's W2D_tf_broadcaster node
  5. DroneNavigationPackage's ControlCenter node
  
In order to send the drone to it's destination, type the following commands in the terminal (it is useful to create these aliases in the
.bashrc file):

        1. alias takeoff='rostopic pub /ControlCenter/external_intervention DroneNavigationPackage/Intervention "EMERGENCY_EXIT_HAPPENED: false
        READY_TO_START: true
        LAND_DRONE: false" '
        
        2. alias calibrate='rostopic pub /ControlCenter/drone_operations std_msgs/Int8 "data: 1" '
        3. alias calculate='rostopic pub /ControlCenter/drone_operations std_msgs/Int8 "data: 2" '
        4. alias travel='rostopic pub /ControlCenter/drone_operations std_msgs/Int8 "data: 3" '

Additional commands:

        1. alias land='rostopic pub /ControlCenter/external_intervention DroneNavigationPackage/Intervention "EMERGENCY_EXIT_HAPPENED: false
        READY_TO_START: false
        LAND_DRONE: true" '      -> land the drone

        2. alias emergency_landing='rostopic pub /ControlCenter/external_intervention DroneNavigationPackage/Intervention "EMERGENCY_EXIT_HAPPENED: true
        READY_TO_START: false
        LAND_DRONE: false" '     -> emergency landing (turning failed, target not reached, ...)
        
        3. alias drone_reset='rostopic pub /ardrone/reset std_msgs/Empty "{}"'    -> reset the drone

        
        
v1.1: A travelling corridor has been defined around the travel-vector. The main goal of this area is to detect if the drone is left the "restricted flying area". In this case, the drone is forced to land. This feature is the basis of a PID control.


