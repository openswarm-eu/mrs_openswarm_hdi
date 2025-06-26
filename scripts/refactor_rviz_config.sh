#!/bin/bash

PACKAGE_PATH=$(rospack find mrs_openswarm_real)

cp $PACKAGE_PATH/tmux/just_flying/rviz/default.rviz /tmp/default.rviz

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/default.rviz