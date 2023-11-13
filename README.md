trakstar_ros
============

ros driver for trakstar magnetic tracker

Instructions to run (11/10/2023, AEC):

```
roscore
roslaunch trakstar.launch (make sure trakstar power is on and status light is a constant green)
```

you're done! Transforms should be published on topic /tf. Check rviz.
Transforms are trakstar# (sensor number) relative to trakstar_base (transmitter).

You can ignore /trakstar_msg and /trakstar_raw_msg for now.
