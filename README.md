---
trakstar_ros
---

ROS driver for trakstar magnetic tracker

---

### Instructions for installing first time*:

  1. make sure libusb-1.0-0-dev and libusb-dev are installed
```
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libusb-dev
```
  2. copy contents of 'config/99-trakstar.rules' to /lib/udev/rules.d/99-libusb.rules:
  - 2.1 cd into trakstar_ros/config and run:
```
sudo cp 99-trakstar.rules /lib/udev/rules.d/99-libusb.rules
```
  - 2.2 **reboot**.

  3. If libusb is still giving you issues, see notes at bottom of https://github.com/ChristophJud/ATC3DGTracker readme.
  - 3.1 run command 'rmmod ehci_hcd'.
  - 3.2 compile demo script from the installation instructions in https://github.com/ChristophJud/ATC3DGTracker

note: usually steps 1 and 2 are enough

---

### Instructions to run (11/10/2023, AEC):

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

Latest (02/12/2024, JP): we ran it in the bimanual manipulation machine. It required steps 3.1 and 3.2.

Latest (03/30/2025, RW): Setup possible for ROS within virtual env (Robostack Mamba)
```
conda install -c conda-forge libusb-compat
```
When building within virtual environments, install libusb-compat via conda (mamba preferred) and do NOT install any libusb packages via apt. They conflict with one another, and virtual environments don't play nice with apt packages. At the time of installation, the latest version on conda-forge is 0.1.12.

Note: the trakstar driver code was written over 10 years ago. libusb was a much earlier version (0.1.*) and some methods have been removed/renamed/deprecated. usb_close() was one of them (renamed to libusb_close). The reason why we needed to install libusb-dev was because it contained previous version of the package (this is my educated guess based on reading up related errors). 

copy the 99-trakstar.rules as described above, and reboot. Then you should be able to catkin_make

Note: I did not run into any permission issues. Not sure if related but: when I first set up the current machine, I had to deal with a usb permission issue (ttyACM0 Permission Error). The solution was to add the current user to the dialout group to obtain permission for port access. If you run into permission errors, ask RW for detail on how to create dialout groups

