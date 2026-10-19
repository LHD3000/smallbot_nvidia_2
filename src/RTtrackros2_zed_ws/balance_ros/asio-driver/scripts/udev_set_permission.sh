#/bin/bash
echo "add group devop"
sudo groupadd devop
sudo usermod -a -G devop $USER

echo "please copy ./scripts/95-device.rules to /etc/udev/rules.d/95-device.rules"
echo "do you want script to do this?[yes/no]"

read result
if [ $result == yes ] ; then
    sudo cp ./scripts/95-device.rules /etc/udev/rules.d/95-device.rules
fi

echo "reload rules"
sudo udevadm control --reload-rules
sudo udevadm trigger