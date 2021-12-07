#!/bin/bash

echo ""
echo "This script copies a udev rules to /etc to facilitate bringing up the usb2com connection."
echo ""

sudo cp `rospack find agv`/udev_rules/agv_usb.rules /etc/udev/rules.d/agv_usb.rules

echo "Reloading rules"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger

echo "Done!"