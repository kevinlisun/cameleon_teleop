## ROS Joystick handler

The package consist of two nodes. First node, named joy_node, reads data from joystick a sends them to node cameleon_teleop, which process incoming data a sends them to Cameleon robot.

To run this package:
1. Install joystick package: sudo apt-get install ros-kinetic-joy 
