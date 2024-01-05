
!!WORKS IN PROGRESS!!
This ROS2 package implements the simulation of multiple mobile manipulators in gz.  
The package has been tested on ros rolling with gazebo Harmonic.  
The deafult lancher "ros2 launch lampo_description lampo_gz_diff.launch.py" spawns two differential mobile manipulators endowed with an UR10 in a simulated whareouse.  
Up to now only the differential drive works, i'm still trying to fix the omnidirectional but some gz-sim PR are needed.
Due to the fact that gz_ros_control works only with a single robot, a topic hardware interface is used to connect to gz(https://github.com/PickNikRobotics/topic_based_ros2_control).  


[![stiima](docs/sc.png)](https://www.stiima.cnr.it/?lang=en)  