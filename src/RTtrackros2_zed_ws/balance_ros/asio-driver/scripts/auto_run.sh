#!/bin/bash
# init gpio pins
gpio mode 13 out
gpio write 13 0

gpio mode 5 out
gpio write 5 0
gpio mode 7 out
gpio write 7 1
gpio mode 8 out
gpio write 8 0
gpio mode 11 out
gpio write 11 1

echo "GPIO pins initialized">>/home/orangepi/dev/reconfigurable-selfbalance-robot/logs/auto_run.log

ip link set can0 down
ip link set can0 type can bitrate 1000000
ip link set can0 up

ip link set can1 down
ip link set can1 type can bitrate 1000000
ip link set can1 up

echo "CAN bus initialized">>/home/orangepi/dev/reconfigurable-selfbalance-robot/logs/auto_run.log
# Run the main program