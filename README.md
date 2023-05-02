## Xavier NX VIO-quad Instructions
#### Setup Xavier NX companion computer (CC):
- ```README_flash_xavier.md```: Flashing firmware onto A203 Xavier NX carrier board, then switching to boot from SD (for larger storage space).
- ```README_bashrc.md```: Edits needed in the ~/.bashrc file on the Xavier NX in order to source ROS correctly and export necessary variables.
- ```README_software_xavier.md```: Installing software to run [ROVIO](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/155340/1/eth-48374-01.pdf), connect to Intel RealSense cameras, etc.
- ```README_cameras.md```: Calibration and information regarding the d435i and T265 Intel RealSense cameras.
- ```README_fiducial.md```: (optional) Setting up apriltag detection with a CSI camera connected to the Xavier NX.

#### Setup CC & FCU (Kakute H7 v2) communication
- ```README_FCU_setup.md```: Flashing firmware on the Kakute H7 v2, and setting up QGroundControl.
- ```README_UART.md```: Connecting Xavier NX with the Kakute flight controller using UART ports and mavros.
- ```README_mocap.md```: Setting up Motive motion capture and PX4 EKF2, for ground truth state estimation.

#### Taking flight
- ```README_tuning.md```: Tuning PX4 quadcopter
- ```README_simulation.md```: Commands for running experiments in simulation.
- ```README_run_stuff.md```: Commands to run in the Xavier NX terminal to start mocap, ROVIO, MAVLink communication through mavros, and fly the quadcopter.

#### Other
- ```README_useful.md```: Tips and links to useful resources.
- (COMING SOON)```README_assembly.md```: Assembling the base quadcopter.
