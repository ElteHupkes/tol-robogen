#!/bin/bash
# Runs gazebo with the correct plugin path
TEST=$1
shift
NR=$1
shift

if [ $1 ]; then
    GZ=gazebo
else
    GZ=gzserver
fi

shift

GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:`pwd`/../../build GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`pwd`/../models $GZ --iters 3000 "$@" stress-$TEST/test_$NR.world
