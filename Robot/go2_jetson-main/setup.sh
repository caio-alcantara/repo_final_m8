#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /><NetworkInterface name="wlan0" priority="default" multicast="default" /></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><Peers><Peer address="192.168.0.5"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
export ROS_DOMAIN_ID=0
