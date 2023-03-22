## Things to check when starting Motive:
1. Make sure cameras have solid blue light. Else, enable all cameras in Devices > Tracking > Enable.
2. Devices > Camera Frame Rate set at 100 Hz.
3. Ensure under "streaming" (looks like a wifi symbol) make sure the Local Interface IP is 128.XXX.XXX.XX when using crazyflie (college of engineering). The local ARC lab network is the 192.XXX.X.XXX.
4. If upon opening Motive the cameras are all in a line on the gound plane, go to File, then select the most recent Calibration Result (toward bootom of pane). If it is not available, a calibration may need to be done (see next section).


## Mocap Calibration:
1. Open Camera Calibration panel (looks like magic wand)
2. Make sure calibration wand is hidden, and then within Calibration Tab, select mask visible.
3. Select Start Wanding and make sure to get roughly 10,000 data points per camera, and ensureeach camera FOV is filled.
4. Click calculate, then apply calculation.
5. Click Ground Plane tab. Set ground plane square at the origin, and make it level.
6. In Perspective view, highlight the markers corresponding to the ground plane square.
7. Click Set Ground Plane.
8. Export calibration data (.cal file) in directory "This PC"/Documents/Optitrack/"Session 2023-01-24", for example

## Tips for good camera placement
https://docs.optitrack.com/hardware/camera-placement?title=Camera_Placement

## Accessing Motive Camera Locations
The Motive API will need to be used in order to obtain the camera locations. The following may be helpful:
- https://forums.naturalpoint.com/viewtopic.php?f=69&t=17247
- https://v21.wiki.optitrack.com/index.php?title=Motive_API
- https://v22.wiki.optitrack.com/index.php?title=Motive_Basics#Calibration_files_.28CAL.29

## Create rigid body for vioquad
- First, the quadcopter local x-axis (in direction of d435i camera) should be aligned with motive world x-axis when creating rigid body.
- Next in the rigid body builder, go to ```Edit```, and decrease the height of the center of mass to a physically reasonable value.

## Installing ROS mocap node on Xavier NX
- See http://wiki.ros.org/mocap_optitrack and
- https://docs.px4.io/main/en/ros/external_position_estimation.html and
- https://github.com/wisc-arclab/arclab_vehicles/tree/ACADO-MPC#mocap
1. On Xavier, install the roslint dependency
```
sudo apt install ros-noetic-roslint
```
2. Install mocap_optitrack with custom config file.
```
cd ~/ROS/catkin_ws/src
git clone https://github.com/seqwalt/mocap_optitrack.git
cd ..
catkin build mocap_optitrack
source devel/setup.bash
```
3. In mocap_optitrack config file /opt/ros/noetic/share/mocap_optitrack/config/mocap.yaml:
```
#
# Definition of all trackable objects
# Identifier corresponds to Trackable ID set in Tracking Tools
#
rigid_bodies:
    '21':
        pose: vioquad/pose
        child_frame_id: vioquad/base_link
        parent_frame_id: world
optitrack_config:
        multicast_address: 239.255.42.99
        command_port: 1510
        data_port: 1511
        enable_optitrack: true
```
Note '21' is the Motive Stream ID of the rigid body for the VIO quadcopter.

## Changing network priority from ethernet to wifi
If the Xavier NX is connected to both ethernet (for a sensor) and wifi (for mocap), we need to make sure wifi has a higher network priority so mocap data can be sent.

1. View current network priorities (lower metric = higher priority) with ```route -n```
  - Note if eth0 and wlan0 connections exist, running ```sudo nmcli dev status``` will show an ordered list in which the higher priority connection is on top (eth0 by default).


2. Switch wifi priority higher (lower metric to 100)
```
sudo nmcli dev modify wlan0 ipv4.route-metric 100
```
3. Switch ethernet priority lower (increase metric to 600)
```
sudo nmcli dev modify eth0 ipv4.route-metric 600
```
4. Check priorities are correct:
```
route -n
sudo nmcli dev status
```


## Test mocap in RVIZ
1. In Xavier run:
```
roslaunch mocap_optitrack mocap.launch
```
2. In base station run:
```
export ROS_MASTER_URI=http://xavier_IP_address:11311
rviz
```
In RVIZ, view the pose of the quadcopter.

## Setup EKF2
This will allow for the use of mocap for position control (see https://docs.px4.io/main/en/ros/external_position_estimation.html#ekf2-tuning-configuration).
1. Enable EKF2 for kakute H7 v2, following https://github.com/PX4/PX4-Autopilot/issues/19986. Set:
- SYS_MC_EST_GROUP = ```EKF2```
- SYS_HAS_MAG = ```0``` (i.e. no magnetometer on kakute h7 v2)
- EKF2_MAG_TYPE = ```None```
- EFK2_EV_DELAY = ```0``` (delay handled by mavros)
- EKF2_HGT_REF = ```Vision```
- EFK2_EV_CTRL = ```15``` (horizontal pos, vertical pos, 3D vel, yaw)
- EKF2_GPS_CTRL = ```0``` (disable GPS)
- EKF2_BARO_CTRL = ```Disabled``` (disable barometer)
- EKF2_RNG_CTRL = ```Disable range fusion``` (disable range finder)
- EKF2_EV_POS_X = ```0.248 ft``` (relative to measured d435i camera)
- EKF2_EV_POS_Y = ```-0.03 ft```
- EKF2_EV_POS_Z = ```0.03 ft```
- EKF2_AID_MASK = ```0```
- EKF2_EVA_NOISE = ```2.87 deg``` (decrease angle measurement noise)
- EKF2_EVP_NOISE = ```0.03 ft``` (decrease position measurement noise)

**Note:** look into vision delay potentially: https://docs.px4.io/main/en/ros/external_position_estimation.html#tuning-ekf2-ev-delay
2. Check that the pose data is correct.
- Start the mocap node on Xavier through ssh.
- Permit UART communication:
```
sudo chmod 666 /dev/ttyTHS0
```
- Start the mavlink connection (separate terminal for each command):
```
roslaunch mocap_optitrack mocap.launch
rosrun topic_tools relay /mocap_node/vioquad/pose /mavros/vision_pose/pose
roslaunch mavros px4.launch
```
- In QGC on the Ubuntu base station, go to ```Analyze Tools``` --> ```Mavlink Inspector``` --> ```Odometry```.
- Open a plot for viewing x,y,z and confirm correct behavior, following: https://docs.px4.io/main/en/ros/external_position_estimation.html#check-external-estimate.
