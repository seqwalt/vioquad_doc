## Editing .bashrc for ROS
Add the following to the top of ~/.bashrc file:  
```
# ROS sourcing
source /opt/ros/noetic/setup.bash
source /home/vio-quad/ROS/catkin_ws/devel/setup.bash

# Automatically setup ROS_IP
tmp=$(hostname -I)
ip=$(echo $tmp | cut -d' ' -f 1)
export ROS_IP=$ip

# ROS package path
export ROS_PACKAGE_PATH=/home/vio-quad/ROS/catkin_ws:$ROS_PACKAGE_PATH

```
