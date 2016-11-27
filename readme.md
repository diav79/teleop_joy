###teleop_joy: A simple code to fly Parrot AR.Drone/Bebop/Bebop2 using joystick

clone the repository in your ros workspace and perform 'catkin_make --pkg teleop_joy' to build.

This particular implementation is for Microsoft Xbox 360 Wired Controller for Linux. For different joystick or system, just change the axes and button indices in code as per necessary.

Prerequisite: ardrone_autonomy(https://github.com/AutonomyLab/ardrone_autonomy) / bebop_autonomy(http://bebop-autonomy.readthedocs.io/en/latest/) as per quadcopter

Can also be tried using Gazebo (https://github.com/dougvk/tum_simulator).

Once the autonomy (or simulation) is on, run

rosrun joy joy_node

Then in different tab, run

rosrun teleop_joy this_joy_ar 
(for AR.Drone; or rosrun teleop_joy this_joy_bebop for Bebop/Bebop2)

The joystick mapping are as follows --

Y button -> take off

B button -> land

X button -> reset

Left Stick up/down axis -> move quad forward/backward

Left Stick left/right axis -> move quad to left/right

Right Stick left/right axis -> turn quad left/right

Direction pad up/down -> increase/decrease altitude

Change the scaling factors in the code as per requirement (default: 0.5)
