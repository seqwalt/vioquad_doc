# Using apriltags for precision landing
## Get relative pose between apriltag and quadcopter
1. Install packages on Xavier NX
```
cd ~/ROS/catkin_ws/src
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/AprilRobotics/apriltag_ros.git
cd ../
catkin build apriltag apriltag_ros
```
2. Print apriltag from https://github.com/AprilRobotics/apriltag-imgs.
  1. For recursive apriltags (which are good for precision landing), download at least three ```tagCustom48h12``` apriltags. Remember the tag IDs (number in file name).
  2. Scale them up around 1000%, using
  ```
  convert <small_marker>.png -scale <scale_chosen_in_percent>% <big_marker>.png
  ```
  3. In your favorite graphical editor, such as Inkscape, load the three tags, and imbed them inside of each other recursively.
  4. Print it out, and precisely measure the sizes, according to https://github.com/AprilRobotics/apriltag_ros#tag-size-definition.
3. Update tag and setting config files as shown here: http://wiki.ros.org/apriltag_ros/Tutorials/Detection%20in%20a%20video%20stream
4. Make sure the camera parameters are known and being published to ```my_camera_name/camera_info```. Likewise, make sure the camera images are being published on ```my_camera_name/image_raw```.
5. After making sure the ```continuous_detection.launch``` file has correct parameters (topic names etc), run the apriltag detector for streamed video:
```
roslaunch apriltag_ros continuous_detection.launch
```
6. The poses of the apriltag bundle is on the ```/tf``` topic.

## Publish image topic from CSI camera
This project uses a IMX219-160 fisheye camera for the down-facing camera that views the AprilTag landing pad. See camera specs at https://www.waveshare.com/imx219-160-camera.htm. Make sure to calibrate the camera using kalibr, just like was done with the D435i stereo camera.
1. Make sure gscam dependencies are installed
```
sudo apt install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
sudo apt install ros-noetic-camera-info-manager
```
2. Install and build updated gscam package
```
cd ~/ROS/catkin_ws/src
git clone https://github.com/seqwalt/gscam.git
cd ../
catkin build gscam
source devel/setup.bash
```
3. Run gscam node to publish CSI image on the topic ```csi_cam/image_raw```, and camera info on ```csi_cam/camera_info```
```
roslaunch gscam nvidia_csi.launch
```
4. Potential errors:
 - If getting "Failed to create CaptureSession" error, try
```
sudo service nvargus-daemon restart
```
 - If getting "libEGL warning: DRI3: failed to query the version libEGL warning: DRI2: failed to authenticate" with "Could not get gstreamer sample" error, make sure X11 forwarding is not being used, i.e. do not use the ```-X``` option in the ```ssh``` command when connecting to the Xavier NX.
