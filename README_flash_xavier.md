# 1) How to flash Xavier NX (A203 carrier board) with JetPack OS to internal eMMC

The instructions here are specifically for the A203 carrier board, which will hold the Jetson Xavier NX. Additionally, these instructions require the use of a host Linux machine. These instructions will help flash the internal eMMC.

1. Use a size M2 screw to secure the WiFi chip onto the A203 carrier board.
2. Use size M2 screws to secure Jetson Xavier NX board onto A203 carrier board.
3. Follow instructions from https://wiki.seeedstudio.com/reComputer_A203_Flash_System/. Notes:
    - After the "Getting Started" section, follow the "Flashing JetPack OS via Command Line," not the "Flashing JetPack OS via NVIDIA SDK Manager" section.
    - Download the most recent NVIDIA Linux Driver Package (i.e. 35.1 from https://developer.nvidia.com/embedded/jetson-linux-archive), which is compatible with Xavier NX. On the 35.1 version page find and click the "Driver Package (BSP)" and "Sample Root Filesystem" to download the driver files.
    - Choose the driver files corresponding to "Jetson Xavier NX SD", which will later allow for using the SD card for holding the OS.
    - Copy the seeedstudio driver file (for example ```tegra194-p3668-all-p3509-0000.dtb```) into the corresponding folder in the official NVIDIA driver folder.
    - A package may need to be installed on the host Linux machine:  
    ```sudo apt-get install qemu-user-static```
    - To flash the board (takes roughly 10 minutes):
    ```
    sudo ./apply_binaries.sh
    sudo ./flash.sh jetson-xavier-nx-devkit-emmc mmcblk0p1
    ```

# 2) How to enable SD card
First, make sure the steps were followed from section 1).  
NOTE: DO NOT upgrade system until after step 3) is DONE.

## Enable the SD card reader
This section is adapted from https://wiki.seeedstudio.com/J101_Enable_SD_Card/, which is mainly for the Jetson Nano.
1. Clone the repo:  
```git clone https://github.com/Seeed-Studio/seeed-linux-dtoverlays.git```
2. Compile jetson-sdmmc-overlay:  
```cd seeed-linux-dtoverlays```
3. Change "compatible" value in a file. This command differs from the one given in the original link:  
```
sed -i '17s#JETSON_COMPATIBLE#\"nvidia,p3449-0000-b00+p3448-0002-b00\"\, \"nvidia\,tegra194\"#' overlays/jetsonnano/jetson-sdmmc-overlay.dts
```
Note tegra194 is used here, since recall the seeedstudio driver file was ```tegra194-p3668-all-p3509-0000.dtb```.
4. Follow the remaining directions in https://wiki.seeedstudio.com/J101_Enable_SD_Card/.
5. Before rebooting, also follow the steps to increase SD card speed (same url).
6. Reboot

# 3) How to Boot from SD card
Make sure sections 1) and 2) were followed
1. In order to correctly partition SD card (i.e. use all 32 GB, not just part of it), install gparted on the Xavier:  
```
sudo apt-get install gparted
```
2. Open gparted by typing ```gparted``` in the terminal.
3. Navigate to the SD card space in the top-right drop down (should be /dev/mmcblk1p1)
4. Add a new partition for the full SD card space.
5. Apply the changes then exit gparted.
6. Follow steps from https://wiki.seeedstudio.com/J1010_Boot_From_SD_Card/ (including reformatting SD to ext4 step)
7. Reboot after updating the file /boot/extlinux/extlinux.conf. Check that the SD card is mounted at root (use command ```lsblk``` in Xavier).
