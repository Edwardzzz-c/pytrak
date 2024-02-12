---
trakstar_ros
---

ros driver for trakstar magnetic tracker

Instructions for installing first time:
  1. make sure libusb-1.0-0-dev, libusb-dev is installed
  2. copy contents of 'config/99-trakstar.rules' to /lib/udev/rules.d/99-libusb.rules
  3. If libusb is giving you issues, see notes at bottom of https://github.com/ChristophJud/ATC3DGTracker readme
    3.1 run command 'rmmod ehci_hcd'.
    3.2 compile demo script from the installation instructions in https://github.com/ChristophJud/ATC3DGTracker

Instructions to run (11/10/2023, AEC):
(make sure trakstar power is on and status light is green; blinking is fine)
```
roscore
roslaunch trakstar trakstar.launch
```

you're done! Transforms should be published on topic /tf and saved as a rosbag in /collected_data/rosbag. Check rviz.
Transforms are trakstar# (sensor number) relative to trakstar_base (transmitter).

With the rosbag, you can do a couple of things.
Example of replaying a saved rosbag:
```
rostopic echo /tf -b trak_2023-11-13-19-07-33.bag
```

Example of converting a saved rosbag to csv format:
```
rostopic echo /tf -b trak_2023-11-13-19-07-33.bag -p > ~/hand_orthosis_ws/src/trakstar_ros/collected_data/csv/trak_2023-11-13-19-07-33.csv
```

---
Latest (11/17/2023, KSL): we were able to run on katelyn's machine! Does require configuring libusb to work.

Latest (02/12/2024, JP): we run it in the bimanual manipulation machine. It required steps 3.1 and 3.2.

TODO (AEC): write a script to automatically convert and save rosbag to csv upon close.
TODO (KSL): get joint angles from transforms.
