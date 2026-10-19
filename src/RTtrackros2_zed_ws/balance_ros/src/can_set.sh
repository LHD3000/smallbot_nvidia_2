
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 restart-ms 100
sudo ip link set can1 up

sudo ip link set can2 down
sudo ip link set can2 type can bitrate 1000000 restart-ms 100
sudo ip link set can2 up