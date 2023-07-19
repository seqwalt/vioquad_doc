# Gazebo Classic Simulation

## Running the simulation
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
4. Run rovio transform node (uses ground truth until rovio is available)
```
rosrun rotors_rovio_tf est_odom_node
```
5. Make ```map``` and ```odom``` frames align, to ensure the est_odometry topic can be used by PX4
```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map
```
6. Relay est_odometry to the name required by PX4
```
rosrun topic_tools relay /est_odometry /mavros/odometry/out
```
7. Run Rovio (optional)
First start the rovio node:
```
roslaunch rovio rovio_node_px4sim.launch
```
Next start the imu integration node
```
rosrun odom_predictor odom_predictor_node
```
8. Run experiment
```
roslaunch quad_control mpc_control.launch simulation:=true
```

# TODO:

## Setup PX4 params
Set ```EKF2_HGT_REF``` to Vision in QGroundControl.


## Adding cameras to the Iris Quadcopter
#### Setup PX4
#### Monocular down-facing camera
#### VIO Stereo camera
```
cd my_path_to/quad_control/extra/simulation/models/vi_camera/xacro
xacro vi_camera.urdf.xacro > vi_camera.urdf
gz sdf -p vi_camera.urdf > ../vi_camera.sdf
```

## Running ROVIO in sim
1. Create new launch file ```rovio_node_px4sim.launch```
2. Using the same VIO camera from RotorS simulator, so launch file looks like this:
```
sdfasdf
```
