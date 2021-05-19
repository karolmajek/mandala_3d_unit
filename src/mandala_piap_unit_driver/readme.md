# Mandala 3d unit ROS

# Setup connectivity

 - default IP o wrking computer is 192.168.1.100
 - deafult Unit IP is 192.168.1.10
 - default IP of Velodyne is 192.168.1.201

 if computer IP is other than 192.168.1.100 that have to be changed:

# velodyne

Set destination IP in Velodyne webservice. Keep in mind that both dst ip and destination port has to be set both in Velodyne device and `velodyne_sync.launch`

# 3d unit

Log to unit with NetCat and write registers:
| register | role         |
|----------|--------------|
| 60       | VEE_DST_IP_0 |
| 61       | VEE_DST_IP_1 |
| 62       | VEE_DST_IP_2 |
| 63       | VEE_DST_IP_3 |


with `nc 192.168.1.10 8888`
```
wr 63 100
gc   
saveCount      8
configWrite    0
configIsDirty  0
encoder1Res    2048
encoder2Res    2048
encoder3Res    65535
selfIP:        192.168.1.10
maskIP:        255.255.255.0
gatewayIP:     0.0.0.0
dstIP:         192.168.1.100
MAC:           0E:00:00:01:FE:F4
dstPort        14550
homingVel 65535
EEE
RRR
```
Sequence `EEE` saves to EEPROM, sequence `RRR` resets device
