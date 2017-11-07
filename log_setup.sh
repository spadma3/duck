#!/bin/bash
if [ ! -d '/media/logs' ]; then
    echo "USB not mounted.. mounting to /media/logs ... exiting"
    exit
fi
echo -n "institution? (in {UdM,ETHZ,NCTU,TTIC}): "
read INSTITUTION
export $INSTITUTION
