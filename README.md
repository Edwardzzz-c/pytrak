trakstar_ros

ros driver for trakstar magnetic tracker

Instructions to run (11/10/2023, AEC):
(make sure trakstar power is on and status light is a constant green)
```
roscore
roslaunch trakstar trakstar.launch
```

you're done! Transforms should be published on topic /tf and saved in /collected_data, although only via rosbag for now. Check rviz.
Transforms are trakstar# (sensor number) relative to trakstar_base (transmitter).

You can ignore /trakstar_msg and /trakstar_raw_msg for now.

---
Latest (11/13/2023, AEC): launch file now opens and saves a rosbag. Not yet tested for useful data output. 

TODO (AEC): write scripts to convert and save rosbag to csv upon close.
TODO (KSL): get joint angles from transforms.
