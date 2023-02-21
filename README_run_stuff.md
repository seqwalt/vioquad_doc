## IMPORTANT: Turning Xavier NX on/off
**Turning ON:**
- With companion computer power cord disconnected, plug in battery to quadcopter.
- After the FC sends its "ready to go" chime, plug in comapnion computer power cord, and wait around 1 minute before attempting to ssh to Xavier. The ssh command from the Ubuntu base station (which is on the same wifi ARC LAN network), is ```ssh vio-quad@the_vio_quad_IP```.

**Turning OFF:**
- Do `sudo shutdown -P now` command on Xavier NX, then wait about 1 minute before unplugging.

## Start mocap
1. Start the Motive software on mocap computer, and ensure the vioquad rigid body is detected. Make sure the streaming IP in Motive is the 192.XXX.X.XXX address.
2. In Xavier run the following in separate terminals:
```
roslaunch mocap_optitrack mocap.launch
rosrun topic_tools relay /mocap_node/vioquad/pose /mavros/vision_pose/pose
```
3. For visualization on the Ubuntu base station, run:
```
export ROS_MASTER_URI=http://xavier_IP_address:11311
rosrun rviz rviz
```
In RVIZ, view the pose of the quadcopter by clicking ```Add``` --> ```By Topic``` --> ```/mocap_node/vioquad/pose```.

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
After starting mocap, communicate between Xavier NX, Kakute H7 v2 and QGC using mavros:
1. Ensure correct UART wire connection.
2. On Xavier NX run:
```
sudo chmod 666 /dev/ttyTHS0
```
3. Next, start mavros with
```
rosrun mavros mavros_node _fcu_url:=/dev/ttyTHS0:921600 _gcs_url:=udp://@my_base_station_IP
```
OR
```
roslaunch mavros px4.launch
```
4. Start QGroundControl on the base station.

## Start autonomous flight
1. With the RC Transmitter in ```POSITION``` mode, ARM the quadcopter with the transmitter.
2. After starting mocap and/or VIO and mavros, on the Xavier NX run:
```
roslaunch quad_control controller.launch
```
3. **WARNING:** Be ready to "Ctrl - C" the quad_control nodes in case the quadcopter does not behave as expected!
