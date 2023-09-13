# D435i Camera
I used some files from https://github.com/engcang/vins-application/tree/master/ROVIO/d435i. This repo alternates the infrared emitter, such that the stereo cameras could be used for VIO (with out emitter dots) and also depth perception. I had to edit a couple of the files to remove some errors, and have updated my fork accordingly: https://github.com/seqwalt/realsense-ros.

When testing, it was clear their calibration does not match my d435i camera. Instead I used calibration files for d435i that I generated using kalibr in Docker (see section below). After running ROVIO with only 15 features, this worked in real time.

Some info regarding D435i usage:
- https://github.com/IntelRealSense/librealsense/blob/master/doc/d435i.md

# T265 Camera
## Calibration of intel T265 camera
This product is calibrated in the factory, and the camera model matrix and distortion parameters can be determined with:  
```
roslaunch realsense2_camera rs_t265.launch
rostopic echo /camera/fisheye1/camera_info
```
More info regarding distortion models, and the T265 camera specifications:
- https://github.com/ethz-asl/kalibr/issues/298
- https://github.com/ethz-asl/kalibr/issues/17
- https://github.com/IntelRealSense/realsense-ros/issues/679
- https://github.com/IntelRealSense/realsense-ros/pull/1225  
- https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md

### Resizing image outputs
The image outputs were resized to a smaller resolution to try to get ROVIO working with the T265. This did not fix things, but the re-sizing code may be useful later. The updated launch file at https://github.com/seqwalt/realsense-ros/blob/main/realsense2_camera/launch/rs_t265.launch adds two nodelets (see bottom of the file), which reduces resolution of each stereo image.

# Using Kalibr in a Docker Container for the d435i
## Preliminaries
- If using d435i IMU for VIO (i.e. not the FCU IMU) and if not done already make sure to **calibrate the d435i IMU**. Use these resources:
  - https://www.intelrealsense.com/wp-content/uploads/2019/07/Intel_RealSense_Depth_D435i_IMU_Calibration.pdf
  - https://github.com/IntelRealSense/librealsense/blob/master/doc/d435i.md#imu-calibration.
- **Print out an aprilgrid**, and record its parameters into a file called ```aprilgrid.yaml```, following steps from https://github.com/ethz-asl/kalibr/wiki/calibration-targets#a-aprilgrid.
- **Record an IMU-only rosbag** called ```imu.bag```, used later for calculating allan variance parameters. Place your IMU on some damped surface and record your IMU data to a rosbag. You must record at least 3 hours of data. The longer the sequence, the more accurate the results.
  - If using FCU IMU instead of d435i IMU for VIO, record the rosbag by:
    1. Plug FCU into base station with usb<->usb-c cord and determine usb port used (use ```ls \dev``` with the cord plugged in and not plugged in to determine port). Most likely ```ttyACM1``` or ```ttyACM0```.
    2. Start ```mavros``` on the base station (make sure QGC is closed). If the port is ```ttyACM1``` do:
    ```
    rosrun mavros mavros_node _fcu_url:=/dev/ttyACM1:921600
    ```
    where ```921600``` is the baud rate.
    3. Record the imu rosbag of the topic ```/mavros/imu/data_raw```.
- **Record a rosbag** called ```vioquad_d435i.bag``` containing the IMU and both infrared stereo camera topics. The camera needs to be facing the aprilgrid, as shown in the video here: https://github.com/ethz-asl/kalibr/wiki#tutorial-IMU-camera-calibration

## Setup Docker
1. Get Docker: https://docs.docker.com/get-docker/
2. Build the kalibr docker and call it ```vio_calib```. The Docker file with ROS 18.04 was used to avoid some issues when trying 20.04.
```
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t vio_calib -f Dockerfile_ros1_18_04 .
```
3. Make a directory on the base station called ```cam_imu_calib``` that will hold ```aprilgrid.yaml```, ```vioquad_d435i.bag```, and ```imu.bag```. The Docker container will be able to access the contents of this directory.
```
cd yourPath
mkdir -p cam_imu_calib/d435i cam_imu_calib/allan_var cam_imu_calib/rovio
cd cam_imu_calib
mv path/to/aprilgrid.yaml .
mv path/to/vioquad_d435i.bag d435i
mv path/to/imu.bag allan_var
```
4. Mount a folder called ```data``` for use by the Docker container:
```
export FOLDER=yourPath/cam_imu_calib
xhost +local:root
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" vio_calib
```

## Commiting changes to a Docker container
- If we make any changes within the Docker container (e.g. install something with ```apt```), when we exit the Docker container (by typing ```exit```), then next time we open the Docker container our changes will not be there.
- As such, after making a change then exiting Docker, we need to commit our changes. Once out of the Docker container, enter
```
sudo docker ps -a
```
to display a list of launched containers. The most recent launch will be on the first line, and look something like:
```
CONTAINER ID   IMAGE        COMMAND                  CREATED              STATUS                      PORTS     NAMES
abcd12345678   vio_calib    "/bin/sh -c 'export …"   About a minute ago   Exited (0) 28 seconds ago             charming_darwin
```
- To commit our changes, enter:
```
sudo docker commit abcd12345678 vio_calib
```
- Re-open the container
```
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" vio_calib
```
then confirm your changes are there.

## Compute IMU Allan variance
Following steps from https://github.com/ori-drs/allan_variance_ros.
1. Once in the container, build the ```allan_variance_ros``` github repo:
```
cd /catkin_ws/src
git clone https://github.com/ori-drs/allan_variance_ros.git
cd ..
catkin build
source /catkin_ws/devel/setup.bash
```
2. Reorganize the ROS messages of ```imu.bag``` by timestamp:
```
cd /data
rosrun allan_variance_ros cookbag.py --input d435i/imu.bag \
    --output allan_var/cooked_imu.bag
```
3. Create config file called ```/data/allan_var/d435i.yaml``` then run the Allan variance computation tool.
  - Paste the following into d435i.yaml, then update the values to reflect your ```imu.bag```:
```
imu_topic: "/camera/imu"
imu_rate: 400        # default rate for d435i
measure_rate: 100    # Rate to which imu data is subsampled (set to ~1/4 of imu_rate)
sequence_time: 10800 # 3 hours in seconds (length of imu.bag)
```
  - Run Allan variance computation.
```
cd /data/allan_var
rosrun allan_variance_ros allan_variance . d435i.yaml
```
4. Generate the ```imu.yaml``` needed by kalibr:
```
cd /data/allan_var
rosrun allan_variance_ros analysis.py --data allan_variance.csv
```
5. Withing the generated ```imu.yaml```, update the ```rostopic``` and ```update_rate``` values. See https://github.com/ethz-asl/kalibr/wiki/yaml-formats#imu-configuration-imuyaml for more guidance.
4. If exiting Docker, remember to **commit your changes** as described in the previous section.

## Calibrate the D435i camera & IMU system
1. Once in the docker container, run the following to do stereo calibration:
```
source /catkin_ws/devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/d435i/vioquad_d435i.bag \
    --target /data/aprilgrid.yaml \
    --models pinhole-radtan pinhole-radtan \
    --topics /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw
```
Note for the CSI fisheye camera, the pinhole-equidistant model was used instead, with the command: ```rosrun kalibr kalibr_calibrate_cameras --bag /data/csi_cam/csi_cam_raw.bag --target /data/aprilgrid.yaml --models pinhole-equi --topics /csi_cam/image_raw```.
2. The previous step should generate a file called ```vioquad_d435i-camchain.yaml```. To now do stereo-IMU calibration, run:
```
rosrun kalibr kalibr_calibrate_imu_camera \
    --target /data/aprilgrid.yaml \
    --cam /data/d435i/vioquad_d435i-camchain.yaml \
    --imu /data/allan_var/imu.yaml \
    --bag /data/d435i/vioquad_d435i.bag
```

## Create ROVIO calibration files
1. The previous step should generate **another** file called ```vioquad_d435i-camchain-imucam.yaml```.
To use this file to generate rovio calibration files run:
```
cd /data/rovio
rosrun kalibr kalibr_rovio_config --cam /data/d435i/vioquad_d435i-camchain-imucam.yaml
```
2. Send the ROVIO files from Docker to Jetson Xavier NX.  While in the Docker container:
```
scp rovio* vio-quad@my_xavier_IP_address:/path/to/rovio/cfg/xavier_nx_d435i
```
This is just an example, more general usage is
```
scp source/file1 source/file2 \
    destination_user@destination_IP:folder/to/save/files/in
```
