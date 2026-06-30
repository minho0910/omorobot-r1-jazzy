#!/bin/bash

echo "Motor Driver (USB Serial from RS232) : /dev/ttyUSBx to /dev/ttyMCU or /dev/ttyMotor :"
if [ -f "/etc/udev/rules.d/98-omo-r1-mcu.rules" ]; then
    echo "98-omo-r1-mcu.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="3-1", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMCU"' | sudo tee /etc/udev/rules.d/98-omo-r1-mcu.rules
    echo '98-omo-r1-mcu.rules created'
fi
if [ -f "/etc/udev/rules.d/98-omo-r1-mcu-1.rules" ]; then
    echo "98-omo-r1-mcu-1.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="3-1", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMotor"' | sudo tee /etc/udev/rules.d/98-omo-r1-mcu-1.rules
    echo '98-omo-r1-mcu-1.rules created'
fi
if [ -f "/etc/udev/rules.d/98-omo-r1-mcu-2.rules" ]; then
    echo "98-omo-r1-mcu-2.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="1-3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMCU"' | sudo tee /etc/udev/rules.d/98-omo-r1-mcu-2.rules
    echo '98-omo-r1-mcu-2.rules created'
fi
if [ -f "/etc/udev/rules.d/98-omo-r1-mcu-3.rules" ]; then
    echo "98-omo-r1-mcu-3.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="1-3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMotor"' | sudo tee /etc/udev/rules.d/98-omo-r1-mcu-3.rules
    echo '98-omo-r1-mcu-3.rules created'
fi


echo "YD LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLIDAR or /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/97-omo-r1-lidar.rules" ]; then
    echo "97-omo-r1-lidar.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="3-3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' | sudo tee /etc/udev/rules.d/97-omo-r1-lidar.rules
    echo '97-omo-r1-lidar.rules created'
fi
if [ -f "/etc/udev/rules.d/97-omo-r1-lidar-1.rules" ]; then
    echo "97-omo-r1-lidar-1.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="3-3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLiDAR"' | sudo tee /etc/udev/rules.d/97-omo-r1-lidar-1.rules
    echo '97-omo-r1-lidar-1.rules created'
fi
if [ -f "/etc/udev/rules.d/97-omo-r1-lidar-2.rules" ]; then
    echo "97-omo-r1-lidar-2.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="1-1", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' | sudo tee /etc/udev/rules.d/97-omo-r1-lidar-2.rules
    echo '97-omo-r1-lidar-2.rules created'
fi
if [ -f "/etc/udev/rules.d/97-omo-r1-lidar-3.rules" ]; then
    echo "97-omo-r1-lidar-3.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", KERNELS=="1-1", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLiDAR"' | sudo tee /etc/udev/rules.d/97-omo-r1-lidar-3.rules
    echo '97-omo-r1-lidar-3.rules created'
fi


echo "@@@@@ reload rules @@@@@"
udevadm control --reload-rules
udevadm trigger
exit 0
