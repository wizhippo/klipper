#!/bin/bash

sudo echo stop > /sys/class/remoteproc/remoteproc0/state

sudo echo start > /sys/class/remoteproc/remoteproc0/state
