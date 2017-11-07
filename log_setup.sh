#!/bin/bash
if [! -d "/dev/sda1" ]; then
    echo "Have you plugged in the USB drive for logging? Exiting without creating log..."
    exit
fi
if [! -d "/media/logs"]; then
    echo "USB not mounted.. mounting to /media/logs"
    sudo mount -t vfat /dev/sda1 /media/logs -o umask=000
fi
echo -n "institution? (in {UdM,ETHZ,NCTU,TTIC}): "
read INSTITUTION

