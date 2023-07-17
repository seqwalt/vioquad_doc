## Install Base ROS Noetic
Follow http://wiki.ros.org/noetic/Installation/Ubuntu. For the installation step, do:  
```
http://wiki.ros.org/noetic/Installation/Ubuntu
```

## Make a catkin workspace
1. Make ROS folder in home directory: ```mkdir ~/ROS```
2. Install catkin packages:  
```
sudo apt install ros-noetic-catkin
sudo apt install python3-catkin-tools
```
3. Make catkin workspace:  
```
cd ~/ROS
mkdir -p catkin_ws/src
```

## Editing .bashrc for ROS
Add the following to the top of the ~/.bashrc file:  
```
# ROS sourcing
source /opt/ros/noetic/setup.bash
source /home/my_user_name/ROS/catkin_ws/devel/setup.bash

# Automatically setup ROS_IP
tmp=$(hostname -I)
ip=$(echo $tmp | cut -d' ' -f 1)
export ROS_IP=$ip

# Exports
export ROS_PACKAGE_PATH=/home/my_user_name/ROS/catkin_ws:$ROS_PACKAGE_PATH  # ROS package path
export PATH=$PATH:/home/vio-quad/.local/bin  # pyserial path (for mavlink_ulog_streaming.py)
```

## Install ROVIO and related packages
1. Install ROVIO dependencies:  
```
sudo apt install libeigen3-dev libyaml-cpp-dev ros-noetic-cv-bridge ros-noetic-tf ros-noetic-image-transport
cd ~/ROS/catkin_ws/src
git clone https://github.com/ANYbotics/kindr
cd ~/ROS/catkin_ws
catkin build
```
2. Clone ROVIO:  
```
git clone https://github.com/seqwalt/rovio -b xavier_nx
cd ~/ROS/catkin_ws/src/rovio
git submodule update --init --recursive
```
NOTE: The xavier_nx branch reduces the image buffer length (to decrease lag during periods of high computation). In file ~/ROS/catkin_ws/src/rovio/include/rovio/RovioNode.hpp, the change can be noted on lines 209 and 210. This change reduces the camera image buffer from 1000 to 50.
3. Build ROVIO
```
cd ~/ROS/catkin_ws
catkin build rovio -DROVIO_NCAM=2 -DROVIO_NMAXFEATURE=15 --cmake-args -DCMAKE_BUILD_TYPE=Release
```
4. Get dependencies then clone and build rotors_rovio_tf:  
```
sudo apt install ros-noetic-tf2-geometry-msgs
cd ~/ROS/catkin_ws/src
git clone https://github.com/seqwalt/rotors_rovio_tf.git
cd ../
catkin build rotors_rovio_tf
```
5. Get dependencies then clone and build odom_predictor:  
```
cd ~/ROS/catkin_ws/src
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/minkindr.git
git clone https://github.com/ethz-asl/minkindr_ros.git
```
```
cd ~/ROS/catkn_ws/src/minkindr
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git
git clone https://github.com/ethz-asl/numpy_eigen.git
git clone https://github.com/ethz-asl/gflags_catkin.git
```
```
sudo apt install ros-noetic-eigen-conversions ros-noetic-tf-conversions
cd ~/ROS/catkin_ws
catkin build
```
6. Turn off visualizations (required for when running ROVIO from ssh). In file /home/vio-quad/ROS/catkin_ws/src/rovio/cfg/xavier_nx_d435i/rovio_stereo.info, set:
```
doFrameVisualisation false;
```

## Install realsense camera drivers
Building from source roughly following: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
1. Update ubuntu distro:  
```
sudo apt-get update && sudo apt-get upgrade
```
2. Prepare Linux backend and dev. environment (with cam unplugged):  
```
sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```
3. Following (almost exactly) from comment by AlbertoOddness in github issue: https://github.com/IntelRealSense/librealsense/issues/10416, we then do:
```
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-setuptools \
    python3-pip \
    python3-dev
sudo apt-get install python3.9-dev
cd ~/
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/
mkdir build && cd build
cmake ../ -DFORCE_RSUSB_BACKEND=false -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
sudo make uninstall && make clean && make -j6 && sudo make install
```
4. Fix "failed to open usb interface error" using issue comment: https://github.com/IntelRealSense/realsense-ros/issues/1408#issuecomment-698128999  
```
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d
```

## Install ROS wrapper for realsense
Install info adapted from: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
1. Enter workspace:  
```
cd ~/ROS/catkin_ws/src
```
2. Clone the latest Intel RealSense ROS1 release and prepare for build:  
```
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
rosdep install --from-paths realsense-ros --ignore-src │ -r -y
sudo apt purge ros-noetic-librealsense2
```
3. Fix errors and build
  1. Fix ```undefined symbol: _ZN2cv3MatC1Ev```. In the appropriate location, add to file ~/ROS/catkin_ws/src/realsense-ros/realsense2_camera/CMakeLists.txt:  
```
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
```
  2. build
```
cd ~/ROS/catkin_ws
catkin build realsense2_camera realsense2_description -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```

## Setup mavlink_ulog_streaming.py (optional)
This is useful for logging ulog data when your FCU does not have an SD card. Instead, a python script from PX4 can stream the data to you companion computer. First we download the script, install dependencies, then make it executable:
```
cd
https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/mavlink_ulog_streaming.py
pip3 install datetime argparse pymavlink pyserial
chmod +x mavlink_ulog_streaming.py
```
Usage can be found in ```README_run_stuff.md```.
## Install miscellaneous packages
```
sudo apt install ros-noetic-rqt-image-view ros-noetic-image-proc
```

## Fix opencv version issue
If you encounter a Segmentation Fault, run the ros node with gdb, and see if opencv is at fault. If it is, this section might help you.  
There was an issue where ROVIO was not working because the ros noetic opencv (version 4.2) was not the same version of opencv that came with the JetPack OS (version 4.5).  
1. Remove all opencv packages:  
```
sudo apt purge libopencv* python3-opencv
```
2. Comment out lines in the nvidia-l4t-apt-source.list file that grabs packages from online:  
```
sudo nano /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
```
Then comment out all lines, then save and exit.  
3. Install opencv packages with only the ros-latest.list file active:  
```
sudo apt install ros-noetic-opencv-apps*
```
4. While we are here we can re-install the miscellaneous packages from prev section (in case they were uninstalled in the process):  ```
5. Re-activate nvidia-l4t-apt-source.list file by undo-ing the comments that were added.  
NOTE: Now everytime a ```sudo apt upgrade``` command is run, on of the outputs of this command may be  
```
The following packages have been kept back:
  libopencv-dev
```
This is fine though, and nothing to worry about.
6. Re-build the catkin_ws now

## Rebuilding catkin_ws
If for some reason the catkin_ws needs to be rebuilt from the scratch, do:  
```
cd ~/ROS/catkin_ws
catkin clean --deinit -y
catkin init
catkin build rovio -DROVIO_NCAM=2 -DROVIO_NMAXFEATURE=15 --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build realsense2_camera realsense2_description -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin build quad_control
catkin build
```
