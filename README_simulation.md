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
4. Rectify down-facing camera image (only if using fisheye camera)
```
ROS_NAMESPACE=fisheye_camera rosrun image_proc image_proc
```
5. Run experiment
```
roslaunch quad_control mpc_control.launch simulation:=true
```

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
