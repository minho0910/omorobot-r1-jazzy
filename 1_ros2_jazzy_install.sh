#!/bin/bash

echo "@@@@@ Set locale @@@@@"
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings


echo "@@@@@ Setup Sources @@@@@"
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb


echo "@@@@@ Install ROS 2 packages @@@@@"
sudo apt update
sudo apt upgrade -y
sudo apt install -y \
	ros-jazzy-desktop \
	ros-dev-tools \
	nano net-tools openssh-server \
	python3-pip \
	ros-jazzy-navigation2 \
	ros-jazzy-nav2-bringup \
	ros-jazzy-cartographer-ros \
	python3-colcon-common-extensions \
	ros-jazzy-joint-state-publisher-gui \


echo "@@@@@ Sourcing the setup script @@@@@"
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc


echo -e "\033[31m"ros2 jazzy installation is done"\033[0m"
echo -e "\033[31m"system is rebooting in 5sec"\033[0m"
sleep 1
echo -e "\033[31m"system is rebooting in 4sec"\033[0m"
sleep 1
echo -e "\033[31m"system is rebooting in 3sec"\033[0m"
sleep 1
echo -e "\033[31m"system is rebooting in 2sec"\033[0m"
sleep 1
echo -e "\033[31m"system is rebooting in 1sec"\033[0m"
sleep 1
sudo reboot now
exit 0
