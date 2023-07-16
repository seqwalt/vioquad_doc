# Useful Resources

## General
- Benchmarks of VIO algorithms on Jetson computers:
  - https://arxiv.org/abs/2103.01655
- Useful VIO/ROS/Xavier NX repositories:
  - https://github.com/zinuok?tab=repositories
- Implemenations of different VIO algorithms (see different branches for different cameras)
  - https://github.com/engcang/vins-application

## Jetson Xavier NX
- Documentation for JetPack 35.1:
  - https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/.
  - Even though a different carrier board is used than the Xavier Development carrier board, most of the documentation is still applicable.
- Fan control:
  - https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html#fan-profile-control
  - https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html#fan-speed-control
- Overclocking:
  - https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html#maximizing-jetson-xavier-performance
- Change wifi via terminal:
  - https://askubuntu.com/questions/1164074/how-to-connect-to-wifi-using-just-the-terminal
- Instructions for logging into github on the Xavier NX without needing a token
  - Log into github, so ```git push``` doesn't ask for password:  
  ```
  gh auth login
  ```
  Follow prompt: Github.com -> HTTPS -> Browser option
  - Log out:
  ```
  gh auth logout
  ```

## PX4
- PX4 System Architecture:
  - https://docs.px4.io/main/en/concept/px4_systems_architecture.html

## QGroundControl (QGC)
- QGC console commands:
  - https://docs.px4.io/main/en/modules/modules_main.html

## Changing network priority from ethernet to wifi

1. View current network priority (lower metric = higher priority):
```
route -n
```
Note if eth0 and wlan0 connects exist running ```sudo nmcli dev status``` will show an ordered list in which the higher priority connection is on top (eth0 by default).  

2. Switch wifi priority higher (metric lower)
```
sudo nmcli dev modify wlan0 ipv4.route-metric 100
```

3. Switch ethernet priority lower (metric higher)
```
sudo nmcli dev modify eth0 ipv4.route-metric 600
```
4. Check priorities are correct:
```
route -n
sudo nmcli dev status
```
