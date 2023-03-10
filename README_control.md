## Auto-tune PID gains
Follow https://docs.px4.io/main/en/config/autotune.html. If autotune fails, in QGC increase the ```MC_AT_SYSID_AMP``` parameter by steps of one, and re-test the autotune after each change.

Current Rate gains:
- thrust curve: 0.3
- rollrate_k: 2.15, rollrate_d: 0.0018, rollrate_i: 0.077
- pitchrate_k: 2.00, pitchrate_d: 0.0020, pitchrate_i: 0.084

## Control vioquad (gazebo sim)
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
cd ~/PX4-Autopilot
make px4_sitl gazebo
roslaunch quad_control controller.launch
```

## Control vioquad (mocap pose data)


## Control vioquad (VIO pose data)
