#! /bin/bash

# please make sure the can0 is up and bitrate is correct.

PASSWORD=hkclr
PHY_CAN_INTERFACE=can0
VIR_CAN_TUNNEL_0=vxcan0
VIR_CAN_TUNNEL_1=vxcan1

CONTAINER_NAME=packaging_machine_comm
WS_NAME=packaging_system

ROS_DISTRO=humble

while true; do
    # Check if the container is running
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        # If the container is running, get its PID
        DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' $CONTAINER_NAME)
        echo "Container is running. PID: $DOCKERPID"
        break  # Exit the loop if the container is up
    else
        # If the container is not running, print a message and wait
        echo "Container $CONTAINER_NAME is not running. Checking again in 1 seconds..."
        sleep 1  # Wait for 1 seconds before checking again
    fi
done

echo $PASSWORD | sudo -S ip link add $VIR_CAN_TUNNEL_0 type vxcan peer name $VIR_CAN_TUNNEL_1 netns $DOCKERPID 
echo "Added a $VIR_CAN_TUNNEL_0"

echo $PASSWORD | sudo -S modprobe can-gw 
echo $PASSWORD | sudo -S cangw -A -s $PHY_CAN_INTERFACE -d $VIR_CAN_TUNNEL_0 -e 
echo $PASSWORD | sudo -S cangw -A -s $VIR_CAN_TUNNEL_0 -d $PHY_CAN_INTERFACE -e 
echo $PASSWORD | sudo -S ip link set $VIR_CAN_TUNNEL_0 up 

echo $PASSWORD | sudo -S nsenter -t $DOCKERPID -n ip link set $VIR_CAN_TUNNEL_1 up
echo "Mapped $VIR_CAN_TUNNEL_0 to $VIR_CAN_TUNNEL_1"

docker exec $CONTAINER_NAME /bin/bash -c "\
    source /opt/ros/$ROS_DISTRO/setup.bash; \
    source /$WS_NAME/install/setup.bash; \
    ros2 lifecycle set /lifecycle_manager configure; \
    sleep 3; \
    ros2 lifecycle set /lifecycle_manager activate"