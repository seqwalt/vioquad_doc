## Control vioquad (gazebo sim)
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
cd ~/PX4-Autopilot
make px4_sitl gazebo
rosrun quad_control offb_node
```

## Control vioquad (mocap pose data)


## Control vioquad (VIO pose data)
