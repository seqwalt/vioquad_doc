Note on Kakute H7 V2: It uses a STM32 MCU, which according to https://docs.px4.io/main/en/advanced_config/bootloader_update_from_betaflight.html#flash-bootloader cannot be bricked, so even is flashing fails, new firmware can be flashed.

The following instructions are incorrect, since we have a Kakute H7 V2, while the following is for V1. Ardupilot already supports V2 (https://ardupilot.org/copter/docs/common-holybro-kakuteh7-v2.html), whereas for PX4 development is in the works( and https://docs.holybro.com/fpv-flight-controller/kakute-h7-v2/supported-firmware). Both Ardupilot and PX4 support QGroundControl and can use ROS noetic for mavros.

Noetic mavros install instructions: https://ardupilot.org/dev/docs/ros-install.html#installing-mavros

mavros usage: https://wiki.ros.org/mavros

PX4 send mavros messages: https://docs.px4.io/main/en/ros/mavros_custom_messages.html

# INCORRECT (for V1, but we need V2) Getting PX4 on the Kakute H7 Flight Controller

1. PX4 Bootloader Update
The H7 comes with Betaflight by default, so follow steps here: https://docs.px4.io/main/en/flight_controller/kakuteh7.html#px4-bootloader-update

2. Building Firmware
```
pip3 install kconfiglib future
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot
make holybro_kakuteh7_default
```

3. Installing PX4 firmware
```
cd ~/PX4-Autopilot
make holybro_kakuteh7_default upload
```
Once the terminal says ```Waiting for bootloader...``` plug the quadcopter into the computer.

# Trying branch that supports kakute h7 v2
Following https://github.com/PX4/PX4-Autopilot/pull/20545#issuecomment-1381321693

```
cd ~/
rm -rf PX4-Autopilot
git clone https://github.com/j-chen-opteran/PX4-Autopilot.git -b pr-bmi270-fixed --recursive
cd ~/PX4-Autopilot
make holybro_kakuteh7v2_bootloader
sudo apt install dfu-util
```
Need to flash bootloader. To enter DFU mode, hold the boot button down while connecting the USB cable to your computer. The button can be released after the board is powered up.

```
dfu-util -a 0 --dfuse-address 0x08000000 -D build/holybro_kakuteh7v2_bootloader/holybro_kakuteh7v2_bootloader.bin
```
Reboot the flight controller and it let it boot without holding the boot button.
```
git tag v1.9.0-rc3
make holybro_kakuteh7v2_default
make holybro_kakuteh7v2_default upload
```
Once the terminal says ```Waiting for bootloader...``` plug the quadcopter into the computer. In QGroundControl, set DSHOT_TEL_CFG to "TELEM 1" to enable dshot. Then go to Actuators tab within vehicle setup to correct motor directions.
Known issues:  
1. Parameters are lost after rebooting flight controller. Workaround: https://github.com/PX4/PX4-Autopilot/pull/20545#issuecomment-1381112693
2. cd command not working on the NuttShell

TODO:  
1. Update MOT_POLE_COUNT https://docs.px4.io/main/en/advanced_config/parameter_reference.html
2. Figure out how to use altitude mode. The barometer produces data in the mavlink inspector, but cannot arm to altitude mode.
3. Maybe try Ardupilot, since this flight controller is already supported.
