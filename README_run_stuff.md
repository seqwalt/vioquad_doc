## IMPORTANT: Turning Xavier NX on/off
**Turning ON:**
- With companion computer power cord disconnected, plug in battery to quadcopter.
- After the FC sends its "ready to go" chime, plug in comapnion computer power cord. It can take anywhere from 1-5 mins to boot up. When the Xavier NX fan audibly spins up (audible from a few feet a way), that means you can ssh soon (wait till the noise dies down befor attempting ssh). The ssh command from the Ubuntu base station (which is on the same wifi ARC LAN network), is
```
ssh vio-quad@the_vio_quad_IP
```

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
rviz
```
In RVIZ, view the pose of the quadcopter by clicking ```Add``` --> ```By Topic``` --> ```/mocap_node/vioquad/pose```.

## Run down-facing camera driver
```
roslaunch gscam nvidia_csi.launch
```

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

## Record flight data
For thrust mapping, it may be useful to record various mavros messages (run in either base station or xavier):
```
rosbag record /mavros/setpoint_raw/target_attitude /mavros/imu/data /mavros/local_position/pose
```
where ```target_attitude``` contains messages of PX4-normalized thrust. Thrust mapping can then be approximated using python scripts in ```/home/vio-quad/ROS/catkin_ws/src/vioquad_land/extra/log_analysis```

## Start autonomous flight
1. With the RC Transmitter in ```POSITION``` mode, ARM the quadcopter with the transmitter.
2. After starting mocap and/or VIO and mavros, on the Xavier NX run:
```
roslaunch vioquad_land mpc_control_xavier.launch
```
which will run the control nodes and the AprilTag estimation node.
3. **WARNING:** Be ready to "Ctrl - C" the vioquad_land nodes in case the quadcopter does not behave as expected!
