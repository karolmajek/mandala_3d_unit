# mandala_3d_unit
Rotate your Lidar and build 3D maps

# Build - ROS

Run following commands:

```
cd src/
catkin_init_workspace
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

# Connect device

Set manual IPv4 address on your PC: `192.168.1.100`
Unit has a static IP address: `192.168.1.10`
And by default it communicates with host `192.168.1.100`

# Run

To run 3D unit use below commands:

```
source devel/setup.bash
roslaunch src/mandala_piap_unit_driver/mandala_unit_launch/launch/mandala_unit.launch
```

# Pinout

[Pinout in  docs](docs/Pinout.pdf)
