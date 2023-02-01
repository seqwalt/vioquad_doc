## IMPORTANT: Turning Xavier NX on/off
**Turning ON:**
- plug in cord, and wait around 1 min, 15 sec before attempting to ssh to Xavier.

**Turning OFF:**
- Do `sudo shutdown -h now` command on Xavier NX, then wait about 1 minute before unplugging.

## Start mocap
1. In Xavier run:
```
roslaunch mocap_optitrack mocap.launch
```
2. For visualization in base station, run:
```
export ROS_MASTER_URI=http://xavier_IP_address:11311
rosrun rviz rviz
```
In RVIZ, view the pose of the quadcopter.

## Run rovio with the d435i camera on the Xavier NX
1. Start rovio and wait for tests to finish:
```
roslaunch rovio d435i_rovio_node.launch
```
2. In separate terminal start the camera node:
```
roslaunch realsense2_camera rs_d435i.launch
```
3. Gently move the camera until the rovio image is running real-time.

## Start MAVLink Communication
Communicate between Xavier NX, Kakute H7 v2 and QGC using mavros:
1. Ensure correct UART wire connection.
2. On Xavier NX run:
```
roscore
sudo chmod 666 /dev/ttyTHS0
rosrun topic_tools relay /mocap_node/vioquad/pose /mavros/vision_pose/pose
rosrun mavros mavros_node _fcu_url:=/dev/ttyTHS0:921600 _gcs_url:=udp://@base_station_IP
```
3. Or instead run:
```
sudo chmod 666 /dev/ttyTHS0
rosrun topic_tools relay /mocap_node/vioquad/pose /mavros/vision_pose/pose
roslaunch mavros px4.launch
```
