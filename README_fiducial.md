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
4. Make sure the camera parameters are known and being published to my_camera_name/camera_info.
5. After making sure the ```continuous_detection.launch``` file has correct parameters, run the apriltag detector for streamed video:
```
roslaunch apriltag_ros continuous_detection.launch
```
6. The poses of the apriltag bundle is on the ```/tf``` topic.
