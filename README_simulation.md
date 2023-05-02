# Gazebo Classic Simulation
Steps to start simulation, connect to mavros, visualize in rviz, run experiment:

1. Start simulation with gazebo and px4. Change startup settings in run_gazebo-classic.sh
```
cd path/to/quad_control/extra/simulation
./run_gazebo-classic.sh path/to/PX4-Autopilot
```
2. Connect to mavros
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
3. Start visualization
```
cd path/to/quad_control/extra/rviz
rosrun rviz rviz -d paths.rviz
```
4. Run experiment
```
roslaunch quad_control mpc_control.launch simulation:=true
```
