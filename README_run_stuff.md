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
Communicate between Xavier NX and Kakute H7 v2 using mavros:
1. Ensure correct UART wire connection.
2. On Xavier NX run:
```
roscore
sudo chmod 666 /dev/ttyTHS0
rosrun mavros mavros_node _fcu_url:=/dev/ttyTHS0:921600
```
