#! /bin/bash

DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' ros2_humble_dev) 
sudo ip link add vxcan0 type vxcan peer name vxcan1 netns $DOCKERPID 
sudo modprobe can-gw 
sudo cangw -A -s can0 -d vxcan0 -e 
sudo cangw -A -s vxcan0 -d can0 -e 
sudo ip link set vxcan0 up 
sudo nsenter -t $DOCKERPID -n ip link set vxcan1 up

docker exec ros2_humble_dev /bin/bash -c "\
    source /opt/ros/humble/setup.bash; \
    source /packaging_system/install/setup.bash; \
    ros2 lifecycle set /lifecycle_manager configure; \ 
    ros2 lifecycle set /lifecycle_manager activate"