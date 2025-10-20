#!/bin/bash

############## CHANGE #############
export UAV_NAMES="[uav1, uav2, uav3, uav4, uav5, uav6]"
export WORLD_NAME=simulation
export RESET_COMMAND_FLAG=0
export COMPUTER_NAME="asus1"
export SWARM_HEIGHT=3.0

############## CONFIG #############
# Get the hostname
HOSTNAME_VAR=$(hostname)
export UAV_NAME="$HOSTNAME_VAR"
export RUN_TYPE=simulation
export UAV_TYPE=x500
export UAV_MASS=5.367
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
export CUSTOM_CONFIG=./config/custom_config.yaml
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
elif [ "$WORLD_NAME" = "simulation" ]; then
	export RUN_TYPE=simulation
	export DATUM_LATITUDE=47.397743
	export DATUM_LONGITUDE=8.545594
	export DATUM_HEADING=0.0
	export DATUM_LATITUDE_END=47.3977362
	export DATUM_LONGITUDE_END=8.545392
	export DATUM_LATITUDE_START=47.3977422
	export DATUM_LONGITUDE_START=8.5455928
else
	echo "Unknown WORLD_NAME: $WORLD_NAME"
	exit 1
fi

############## ROS BAG ##############
export SYS_ROSBAG_ENABLED=0         # enable / disable bag recording (be careful to NOT run long term experiments without bags!)
export SYS_ROSBAG_SIZE='1024'       # max size before splitting in Mb (i.e. 0 = infinite, 1024 = 1024Mb = 1Gb)
export SYS_ROSBAG_DURATION='8h'
export SYS_ROSBAG_PATH="$HOME/bag_files/openswarm/"

export SYS_ROSBAG_ARGS="
    --regex
    --split
    --size=$SYS_ROSBAG_SIZE
    --duration=$SYS_ROSBAG_DURATION
    --output-prefix=$SYS_ROSBAG_PATH
"
export SYS_ROSBAG_TOPICS="
	/$UAV_NAME/nlink_linktrack_nodeframe2		
	/$UAV_NAME/uwb(.*)
	/$UAV_NAME/mavros(.*)
	/$UAV_NAME/rtk(.*)
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
	/$UAV_NAME/distributedMapping/localPath
	/$UAV_NAME/distributedMapping/globalMap
	/$UAV_NAME/odom
"

export SIM_ROSBAG_TOPICS="
	/$UAV_NAME/lslidar/pcl_filtered
	/$UAV_NAME/mavros/altitude
	/$UAV_NAME/mavros/battery
	/$UAV_NAME/mavros/estimator_status
	/$UAV_NAME/mavros/extended_state
	/$UAV_NAME/mavros/geofence/waypoints
	/$UAV_NAME/mavros/global_position/compass_hdg
	/$UAV_NAME/mavros/global_position/global
	/$UAV_NAME/mavros/global_position/local
	/$UAV_NAME/mavros/global_position/raw/fix
	/$UAV_NAME/mavros/global_position/raw/gps_vel
	/$UAV_NAME/mavros/global_position/raw/satellites
	/$UAV_NAME/mavros/global_position/rel_alt
	/$UAV_NAME/mavros/gpsstatus/gps1/raw
	/$UAV_NAME/mavros/imu/data
	/$UAV_NAME/mavros/imu/data_raw
	/$UAV_NAME/mavros/imu/mag
	/$UAV_NAME/mavros/imu/static_pressure
	/$UAV_NAME/mavros/imu/temperature_imu
	/$UAV_NAME/mavros/local_position/odom
	/$UAV_NAME/mavros/local_position/pose
	/$UAV_NAME/mavros/local_position/velocity_body
	/$UAV_NAME/mavros/local_position/velocity_local
	/$UAV_NAME/mavros/odometry/in
	/$UAV_NAME/mavros/param/param_value
	/$UAV_NAME/mavros/rallypoint/waypoints
	/$UAV_NAME/mavros/rc/in
	/$UAV_NAME/mavros/rc/out
	/$UAV_NAME/mavros/setpoint_raw/target_attitude
	/$UAV_NAME/mavros/state
	/$UAV_NAME/rtk/all_msgs_raw
	/$UAV_NAME/rtk/bestpos
	/$UAV_NAME/rtk/gpgga
	/$UAV_NAME/rtk/gpgsa
	/$UAV_NAME/rtk/gpgst
	/$UAV_NAME/rtk/gpvtg
"

export SIM_ROSBAG_TOPICS_RECORD="
	/$UAV_NAME/mavros/altitude
	/$UAV_NAME/mavros/battery
	/$UAV_NAME/hw_api/velocity
	/$UAV_NAME/hw_api/angular_velocity
	/$UAV_NAME/estimation_manager/odom_main
	/$UAV_NAME/imu_calibration/imu
	/$UAV_NAME/mavros/altitude
	/$UAV_NAME/distance_travelled
	/$UAV_NAME/home_distance
	/$UAV_NAME/rtk/bestpos
"
#export UAV_NAME=uav9
export SIM_ROSBAG_TOPICS_all="
	/$UAV_NAME/lslidar/pcl_filtered
	/$UAV_NAME/mavros/altitude
	/$UAV_NAME/mavros/battery
	/$UAV_NAME/mavros/estimator_status
	/$UAV_NAME/mavros/extended_state
	/$UAV_NAME/mavros/geofence/waypoints
	/$UAV_NAME/mavros/global_position/compass_hdg
	/$UAV_NAME/mavros/global_position/global
	/$UAV_NAME/mavros/global_position/local
	/$UAV_NAME/mavros/global_position/raw/fix
	/$UAV_NAME/mavros/global_position/raw/gps_vel
	/$UAV_NAME/mavros/global_position/raw/satellites
	/$UAV_NAME/mavros/global_position/rel_alt
	/$UAV_NAME/mavros/gpsstatus/gps1/raw
	/$UAV_NAME/mavros/imu/data
	/$UAV_NAME/mavros/imu/data_raw
	/$UAV_NAME/mavros/imu/mag
	/$UAV_NAME/mavros/imu/static_pressure
	/$UAV_NAME/mavros/imu/temperature_imu
	/$UAV_NAME/mavros/local_position/odom
	/$UAV_NAME/mavros/local_position/pose
	/$UAV_NAME/mavros/local_position/velocity_body
	/$UAV_NAME/mavros/local_position/velocity_local
	/$UAV_NAME/mavros/odometry/in
	/$UAV_NAME/mavros/param/param_value
	/$UAV_NAME/mavros/rallypoint/waypoints
	/$UAV_NAME/mavros/rc/in
	/$UAV_NAME/mavros/rc/out
	/$UAV_NAME/mavros/setpoint_raw/target_attitude
	/$UAV_NAME/mavros/state
	/$UAV_NAME/rtk/all_msgs_raw
	/$UAV_NAME/rtk/bestpos
	/$UAV_NAME/rtk/gpgga
	/$UAV_NAME/rtk/gpgsa
	/$UAV_NAME/rtk/gpgst
	/$UAV_NAME/rtk/gpvtg
	/uav13/lslidar/pcl_filtered
	/uav13/mavros/altitude
	/uav13/mavros/battery
	/uav13/mavros/estimator_status
	/uav13/mavros/extended_state
	/uav13/mavros/geofence/waypoints
	/uav13/mavros/global_position/compass_hdg
	/uav13/mavros/global_position/global
	/uav13/mavros/global_position/local
	/uav13/mavros/global_position/raw/fix
	/uav13/mavros/global_position/raw/gps_vel
	/uav13/mavros/global_position/raw/satellites
	/uav13/mavros/global_position/rel_alt
	/uav13/mavros/gpsstatus/gps1/raw
	/uav13/mavros/imu/data
	/uav13/mavros/imu/data_raw
	/uav13/mavros/imu/mag
	/uav13/mavros/imu/static_pressure
	/uav13/mavros/imu/temperature_imu
	/uav13/mavros/local_position/odom
	/uav13/mavros/local_position/pose
	/uav13/mavros/local_position/velocity_body
	/uav13/mavros/local_position/velocity_local
	/uav13/mavros/odometry/in
	/uav13/mavros/param/param_value
	/uav13/mavros/rallypoint/waypoints
	/uav13/mavros/rc/in
	/uav13/mavros/rc/out
	/uav13/mavros/setpoint_raw/target_attitude
	/uav13/mavros/state
	/uav13/rtk/all_msgs_raw
	/uav13/rtk/bestpos
	/uav13/rtk/gpgga
	/uav13/rtk/gpgsa
	/uav13/rtk/gpgst
	/uav13/rtk/gpvtg
"

############## RVIZ ##############
export USE_RVIZ=1           # enable / disable rviz
export COMPUTER_RVIZ=1      # enable / disable rviz on the computer

########## SWARM FORMATION #######
export SWARM_FORMATION=0 

############## UWB ###############
export SYS_UWB=0 