#! /bin/bash

# please make sure the can0 is up and bitrate is correct.

PHY_CAN_INTERFACE=can0
VIR_CAN_TUNNEL_0=vxcan0
VIR_CAN_TUNNEL_1=vxcan1
CONTAINER_NAME=packaging_system

DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' $CONTAINER_NAME) 
sudo ip link add $VIR_CAN_TUNNEL_0 type vxcan peer name $VIR_CAN_TUNNEL_1 netns $DOCKERPID 
sudo modprobe can-gw 
sudo cangw -A -s $PHY_CAN_INTERFACE -d $VIR_CAN_TUNNEL_0 -e 
sudo cangw -A -s $VIR_CAN_TUNNEL_0 -d $PHY_CAN_INTERFACE -e 
sudo ip link set $VIR_CAN_TUNNEL_0 up 
sudo nsenter -t $DOCKERPID -n ip link set $VIR_CAN_TUNNEL_1 up

docker exec $CONTAINER_NAME /bin/bash -c "\
    source /opt/ros/humble/setup.bash; \
    source /packaging_system/install/setup.bash; \
    ros2 lifecycle set /lifecycle_manager configure; \
    ros2 lifecycle set /lifecycle_manager activate"