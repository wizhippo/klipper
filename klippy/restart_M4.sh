#!/bin/bash

if [ -e "/sys/class/remoteproc/remoteproc0/state" ]; then
    sudo echo stop > /sys/class/remoteproc/remoteproc0/state
    sudo echo start > /sys/class/remoteproc/remoteproc0/state
else
    echo "Nothing!"
fi
