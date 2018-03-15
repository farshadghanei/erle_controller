#!/bin/sh
rosnode kill /controller_node & 
sleep 0.5
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1491, 1495, 1074, 1500, 1072, 0, 0, 0]" &
sleep 1 
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1491, 1495, 1074, 1076, 1072, 0, 0, 0]" 
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [0, 0, 0, 0, 0, 0, 0, 0]" 

