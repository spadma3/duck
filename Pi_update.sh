#!/bin/bash
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
sudo apt update
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
sudo apt -y dist-upgrade
sudo BRANCH=stable rpi-update
sudo apt update
sudo apt dist-upgrade

cd /lib
sudo wget "https://github.com/duckietown/Software/raw/jquack-devel/firmware.tar.gz"
sudo tar -zxvf firmware.tar.gz
sudo rm firmware.tar.gz
sudo reboot
