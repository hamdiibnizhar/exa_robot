# exa_robot
# exa_robot #

Require :

 - ubuntu 16.04 Xenial
 - ROS Kinetic (full desktop config) and its dependency
 - OpenCV 3.4 and its dependency
 - PCL (1.7.2) and its dependency
 - RTABMap SLAM (last) and its dependency

Install its command :

 - $ cd exa_robot/src/command
 - $ sudo chmod a+x link_command exarobot exascript
 - $ ./link_command

This is project for nuclear localization and mapping using mobile robot in unknown hazard environment. this is using stereo camera/depth camera to capturing envrontment in 3D and convert to point cloud.

usage

1. open terminal
2. $ roscore
3. $ exarobot launch freenect	# to activate streo camera
4. $ exarobot launch mapping 	# to mapping
5. $ exarobot launch control 	# to acctivate embedded system to stream nuclear detector data

[Exa_Robot](https://github.com/hamdiibnizhar/exa_robot/tree/master/master_machine/src/results/exa_robot_sideview.jpg)

![](https://github.com/hamdiibnizhar/exa_robot/tree/master/master_machine/src/results/exa_robot_backview.jpg)

Result

Localization and mapping nuclear spots in unknown room
