# Useful Resources

## General
- Benchmarks of VIO algorithms on Jetson computers:
  - https://arxiv.org/abs/2103.01655
- Useful VIO/ROS/Xavier NX repositories:
  - https://github.com/zinuok?tab=repositories
- Implemenations of different VIO algorithms (see different branches for different cameras)
  - https://github.com/engcang/vins-application
- For copying things from terminal easily, these lines can be put in your .bashrc file. Make sure to ```sudo apt install xclip``` first. Note this only works over ssh if the ```-X``` ssh option is used (X11 forwarding). However, X11 forwarding breaks gscam usage it seems.
```
# Helpful general use
alias "copy=head -c -1 | xclip -sel clip" # usage example: pwd | copy    Note this removes the trailing newline character
alias "pwdcp=pwd | copy" # copy pwd to clipboard, removing newline
```

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
