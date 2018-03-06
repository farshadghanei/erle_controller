#!/bin/sh
rosnode kill /controller_node & 
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [0, 0, 0, 0, 0, 0, 0, 0]" 
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [0, 0, 0, 0, 0, 0, 0, 0]" 

