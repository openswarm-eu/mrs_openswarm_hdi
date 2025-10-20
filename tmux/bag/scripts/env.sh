#!/bin/bash

############## CHANGE #############
export UAV_NAMES="[uav6, uav7, uav8, uav9, uav10, uav11, uav12, uav13, uav14, uav15]"
export WORLD_NAME=ingeniarius	# forest, ingeniarius
export RESET_COMMAND_FLAG=0
export COMPUTER_NAME="asus1"
export SWARM_HEIGHT=6.0

############## CONFIG #############
# Get the hostname
HOSTNAME_VAR=$(hostname)
export UAV_NAME="$HOSTNAME_VAR"
export RUN_TYPE=realworld
export UAV_TYPE=ZD700
export UAV_MASS=5.6
export INITIAL_DISTURBANCE_X=0.0
export INITIAL_DISTURBANCE_Y=0.0
export SENSORS="pixhawk"
export OLD_PX4_FW=0
export GPS_PORT="/dev/serial/by-id/usb-Emlid_ReachM+_8243DBF507E2BB50-if02"
export IMU_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
export TOF_UP_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
export RESET_COMMAND="{broadcast: false, command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"

##############  ROS  ##############
# export ROS_MASTER_URI='http://192.168.8.170:11311'
# export ROS_IP='192.168.8.170'
export ROSCONSOLE_FORMAT='[${severity}] [${node}] [${function}] [${line}]: ${message}'

############ MRS FILES ############
export PLATFORM_CONFIG=./config/mrs_uav_system/$UAV_TYPE.yaml
export CUSTOM_CONFIG=./config/custom_config_garmin.yaml
export NETWORK_CONFIG=./config/network_config.yaml

############## GENERAL ##############
# location of the running script
ROS_LAUNCH_PATH="$PWD/launch"
ROS_CONFIG_PATH="$PWD/config"

export ROS_LAUNCH_PATH=$ROS_LAUNCH_PATH
export ROS_CONFIG_PATH=$ROS_CONFIG_PATH

############## GPS DATA ##############
if [ "$WORLD_NAME" = "ingeniarius" ]; then
	export WORLD_CONFIG=./config/worlds/world_local_ingeniarius.yaml
    export DATUM_LATITUDE=41.22061958
    export DATUM_LONGITUDE=-8.52732281
    export DATUM_HEADING=0.0
	export DATUM_LATITUDE_START=41.2209807
	export DATUM_LONGITUDE_START=-8.5272058
	export DATUM_LATITUDE_END=41.221150762330886
	export DATUM_LONGITUDE_END=-8.527161568100276
elif [ "$WORLD_NAME" = "forest" ]; then
	export WORLD_CONFIG=./config/worlds/world_local_forest.yaml
    export DATUM_LATITUDE=41.21711155
    export DATUM_LONGITUDE=-8.52746193
    export DATUM_HEADING=0.0
	export DATUM_LATITUDE_START=41.21683395
	export DATUM_LONGITUDE_START=-8.527359231666667
	export DATUM_LATITUDE_END=41.216510440162686
	export DATUM_LONGITUDE_END=-8.52715607577829
else
    echo "Unknown WORLD_NAME: $WORLD_NAME"
    exit 1
fi

############## ROS BAG ##############
export SYS_ROSBAG_ENABLED=1         # enable / disable bag recording (be careful to NOT run long term experiments without bags!)
export SYS_ROSBAG_SIZE='1024'       # max size before splitting in Mb (i.e. 0 = infinite, 1024 = 1024Mb = 1Gb)
export SYS_ROSBAG_DURATION='8h'
export SYS_ROSBAG_PATH="$HOME/bag_files/$PROJECT_NAME/latest/"

export SYS_ROSBAG_ARGS="
    --regex
    --split
    --size=$SYS_ROSBAG_SIZE
    --duration=$SYS_ROSBAG_DURATION
    --output-prefix=$SYS_ROSBAG_PATH
"
export SYS_ROSBAG_TOPICS="
	/$UAV_NAME/mavros.*
	/$UAV_NAME/rtk.*
	/$UAV_NAME/witmotion_imu/imu
	/$UAV_NAME/hw_api/imu
	/$UAV_NAME/hw_api/gnss
	/$UAV_NAME/hw_api/altitude
	/$UAV_NAME/hw_api/odometry
	/$UAV_NAME/hw_api/velocity
	/$UAV_NAME/hw_api/angular_velocity
	/$UAV_NAME/hw_api/battery_state
	/$UAV_NAME/gnss_verifier/gnss_wstatus
	/$UAV_NAME/lslidar/pcl_filtered
	/$UAV_NAME/estimation_manager/odom_main
	/$UAV_NAME/odometry/gps
	/$UAV_NAME/lio_sam/imupreintegration/odom
"

############## RVIZ ##############
export USE_RVIZ=0           # enable / disable rviz
export COMPUTER_RVIZ=0      # enable / disable rviz on the computer
export DISABLE_ROS1_EOL_WARNINGS=1

########## SWARM FORMATION #######
export SWARM_FORMATION=1

############## UWB ###############
export SYS_UWB=0 