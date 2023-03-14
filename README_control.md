## Tuning PID gains
Below are options for tuning the PID controller for the quadcopter.
### Auto-tuning
Follow https://docs.px4.io/main/en/config/autotune.html. If autotune fails, in QGC increase the ```MC_AT_SYSID_AMP``` parameter by steps of one, and re-test the auto-tune after each change. If it still doesn't work, do manual tuning.

### Manual Tuning (for fine tuning)
Follow https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter_basic.html and optionally https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html

Current rate gains:
- thrust curve: 0.3
- rollrate_k: 2.15, rollrate_d: 0.0018, rollrate_i: 0.077
- pitchrate_k: 2.00, pitchrate_d: 0.0020, pitchrate_i: 0.084
- yawrate_k:1.7, yawrate_i:0.0733

Current attitude gains:
- roll_p: 10.5, pitch_p: 7.5; yaw_p: 6.5

Current velocity gains:
- xy_vel_p_acc: 5, xy_vel_i_acc: 3, xy_vel_d_acc: 0.25
- z_vel_p_acc: 7, z_vel_i_acc: 2, z_vel_d_acc: 0.05

## Control vioquad (gazebo sim)
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
cd ~/PX4-Autopilot
make px4_sitl gazebo
roslaunch quad_control controller.launch
```

## Control vioquad (mocap pose data)
TODO. For now, see ```README_run_stuff.md```.

## Control vioquad (VIO pose data)
TODO
