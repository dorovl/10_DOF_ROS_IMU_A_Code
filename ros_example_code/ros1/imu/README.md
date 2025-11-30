### Depends on installation
    sudo apt update && sudo apt install ros-noetic-serial  ros-noetic-imu-tools
### How to use

Place the folder in the ROS workspace(ros_ws/src/imu)， and turn to the workspace folder(ros_ws/):

    catkin_make
    source devel/setup.bash

The premise is that roscore is started or other launch files are started as the core.
just connect imu :

    rosrun imu imu_node _port_name:=/dev/ttyUSB0

view with rviz :

    roslaunch imu imu_view.launch port_name:=/dev/ttyUSB0

