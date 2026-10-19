sudo chmod 666 /sys/class/gpio/export
sudo chmod 666 /sys/class/gpio/unexport

sudo usermod -aG gpio $USER