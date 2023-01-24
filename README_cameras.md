# T265 Camera

## Calibration of intel T265 camera
This product is calibrated in the factory, and the camera model matrix and distortion parameters can be determined with:  
```
roslaunch realsense2_camera rs_t265.launch
rostopic echo /camera/fisheye1/camera_info
```
See there links for more info:
- https://github.com/ethz-asl/kalibr/issues/298
- https://github.com/ethz-asl/kalibr/issues/17
- https://github.com/IntelRealSense/realsense-ros/issues/679
- https://github.com/IntelRealSense/realsense-ros/pull/1225  
- https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#sensor-origin-and-coordinate-system  
This user has usefule repos:  
- https://github.com/zinuok?tab=repositories  
This repo is useful (see different branches for different cameras):  
- https://github.com/engcang/vins-application  
Benchmarks of VIO algorithms on Jetson computers:  
- https://arxiv.org/abs/2103.01655  

### Resizing image outputs
The image outputs we resized to a smaller resolution to try to get things working. This did not fix things, but the re-sizing code may be useful later. Basically it adds a nodelet at the bottom of the ~/ROS/catkin_ws/src/realsense-ros/realsense2_camera/launch/rs_t265.launch file.

# D435i Camera
As an initial test, I used the files from https://github.com/engcang/vins-application/tree/master/ROVIO/d435i. A benefit of this was that this repo figured out how to disable the infrared emitter, such that the stereo cameras could be used for VIO instead of depth perception. I had to edit a couple of the files to remove some errors. When testing, it was clear their calibration does not match my d435i camera. Instead I used calibration files for d435i that I previoously obtained using the kalibr toolbox. After running ROVIO with only 15 features, this worked in real time.
