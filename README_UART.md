# UART connection between Jetson Xavier NX and Kakute H7 v2 flight controller
The universal asynchronous receiver-transmitter (UART) hardware protocol is used to communicate between the companion computer (Xavier NX) and the flight controller (Kakute H7 v2).
## Check UART Configuration on Xavier NX
According to the pin description of the A203 carrier board being used (https://files.seeedstudio.com/products/103110043/A203%20V2%20pin%20description.pdf), pins 8 and 10 (within 40 pin header) are the transmitter and receiver respectively. Note that the "on" signal will be 3.3V . To make sure these pins are configured properly by the Xavier, see the appropriate documentation:
- Go to [Jetson Linux Archive](https://developer.nvidia.com/embedded/jetson-linux-archive) and click Jetson Linux Version 35.1 since that is the driver version being used (see ```README_flash.md```).
- Click "Jetson Linux Developer Guide" (https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/index.html)
- Navigate to "Configuring the Jetson Expansion Headers" (https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html)

Using the above documentation, we can open a terminal on the Xavier NX, and launch the header configurator:
```
sudo /opt/nvidia/jetson-io/jetson-io.py
```
Click "Configure Jetson 40pin Header" and make sure pins 6, 8 and 10 are set to "GND", "uarta" and "uarta" respectively. If pins need a custom configuration, look into making a device tree overlay (https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html#creating-a-custom-device-tree-overlay).

Disable the ```nvgetty``` service that starts a console on port /ttyTHS0 upon startup, then reboot (https://github.com/JetsonHacksNano/UARTDemo):
```
systemctl stop nvgetty
systemctl disable nvgetty
udevadm trigger
reboot
```
Note to check if a service called serviceName is running do ```systemctl status serviceName.service```.
## Test UART on Xavier NX
Install picocom: ```sudo apt install picocom```. Display ttyTHS ("Tegra High Speed") ports: ```ls -l /dev/ttyTHS*```. Note port /dev/ttyTHS0 is configured by default to associate with the board's UART pins (8 and 10). Connect pins 8 and 10 with a single wire to do a loopback test. Open port with picocom, and activate local copy:
```
sudo picocom /dev/ttyTHS0
# press and hold "Ctrl" then "a". Still holding, press "c" and then release all.
```
Type characters into the terminal, and they should be repeated back to you. This signifies success of the loopback test.
## Configure UART on Kakute H7 v2
By default, PX4 has TELEM1 as the configuration name for the R1, T1 pins i.e. UART1 RX and TX (https://docs.px4.io/main/en/flight_controller/kakuteh7.html). In QGroundControl (QGC) with the quad plugged into the computer, the DSHOT_TEL_CFG was changed from TELEM1 to TELEM2 (maybe not necessary). This frees us to use TELEM1 for the MAVLink connection.

TELEM1 is used as the default MAVLink serial port in QGC (check parameter MAV_0_CONFIG). See https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#mavlink for more MAVLink parameters. According to https://docs.px4.io/main/en/flight_controller/kakuteh7.html#serial-port-mapping, the serial port mapping for TELEM1 is /dev/ttyS0, which can be confirmed in QGC console using ```mavlink status``` (this prints info regarding MAVLink ports such as port location and baud rate).

Note TELEM1 is usually used for ground control telemetry stream, but we'll use it to talk with the companion computer instead. Now, set the following parameters in QGroundControl (see example https://docs.px4.io/main/en/peripherals/mavlink_peripherals.html#example):
- DSHOT_TEL_CFG = ```TELEM2``` (as previously mentioned)
- MAV_0_CONFIG = ```TELEM 1``` (should already be done by default)
- MAV_0_MODE = ```Onboard```
- MAV_0_RATE = ```30000 B/s```
- MAV_0_RADIO_CTL = ```Disabled``` (disable mavlink software throttling)
- MAV_PROTO_VER = ```Always use version 2``` (this conforms with default mavros setting)
- SER_TEL1_BAUD = ```921600 8N1``` (set baud rate)

## Test UART on Kakute H7 v2
Perform a UART loopback test:
1. Connect T1 and R1 serial ports with a single wire on the Kakute board.
2. Open QGC console and run ```mavlink status```.
3. Look at the console response. Under "instance \#0", under "Received Messages" there should be many messages sent, with differing "msgid" values.

## Connect Xavier NX and Kakute H7 v2 with MAVLink
### Ensure voltage levels match (measure with multimeter)
A JST-GHR-6P connector is used to access the R1, T1 and ground pins on the Kakute, which will be connected to pins on the Jetson Xavier NX. Note the voltage level for "on" needs to be the same for the Kakute H7 v2 and Xavier NX. If they are not the same, a logic level converter is necessary: https://www.sparkfun.com/products/12009. NOTE: As previously mentioned, the UART voltage for the pins on the Xavier NX is 3.3V. A multimeter measured a voltage of ~3V between the UART transmitter (specifically T1) and ground. This means **a logic level converter is NOT required**.

### Wire the UART connection
Make the physical connection:
- Connect the Kakute ground to Xavier ground.
- Connect the Kakute UART transmitter to Xavier UART reciever (T1 -> pin 10)
- Connect the Xavier UART transmitter to Kakute UART reciever (pin 8 -> R1)

Check data is being sent:
- On Xavier run ```sudo picocom /dev/ttyTHS0```, and you should see a stream of nonsensical symbols.
- Close picocom.

### Start the MAVLink connection
Mavros and MAVLink will be used, so on the Xavier make sure they are installed (along with some required datasets):
```
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
```

Allow all users to read/write to the Xavier UART device (/dev/ttyTHS0):
```
sudo chmod 666 /dev/ttyTHS0
```

On the Xavier, run the mavros node to obtain a MAVLink connection (https://github.com/mavlink/mavros/blob/master/mavros/README.md#mavros_node----main-communication-node):
```
roscore
rosrun mavros mavros_node _fcu_url:=/dev/ttyTHS0:921600 _gcs_url:=udp://@my_base_station_IP
```
We can see in the Xavier terminal that there was a successful connection, and in the QGC console the command ```mavlink status``` shares that it is receiving data! Woohoo!

### Update launch file and relay mocap pose data
Alternatively, we can start the mavros node with a launch file:
1. On the Xavier, run
```
roscd mavros
sudo nano launch/px4.launch
```
2. Change line 5 from ```<arg name="fcu_url" default="/dev/ttyACM0:57600" />``` to
```
<arg name="fcu_url" default="/dev/ttyTHS0:921600" />
```
3. Similiarly, change line 6 to
```
<arg name="gcs_url" default="udp://@my_base_station_IP" />
```
3. For providing mocap to mavros, run
```
rosrun topic_tools relay /mocap_node/vioquad/pose /mavros/vision_pose/pose
```
4. Launch the node with ```roslaunch mavros px4.launch```
