#!/bin/bash

# UAV6
sleep 12
rosservice call /uav6/control_manager/goto "goal: [13.72, 51.51, 10.5, 0.0]"
sleep 15
rosservice call /uav6/control_manager/goto "goal: [12.63, 63.00, 10.5, 0.0]"
sleep 15
rosservice call /uav6/control_manager/goto "goal: [4.89, 53.00, 10.5, 0.0]"
sleep 15

# # UAV7
# sleep 8
# rosservice call /uav7/control_manager/goto "goal: [9.78, 45.25, 12.0, 0.0]"
# sleep 15
# rosservice call /uav7/control_manager/goto "goal: [9.64, 54.55 , 12.0, 0.0]"
# sleep 15
# rosservice call /uav7/control_manager/goto "goal: [2.00, 46.46, 12.0, 0.0]"
# sleep 15

# # UAV8
# sleep 4
# rosservice call /uav8/control_manager/goto "goal: [16.34, 45.05 , 13.5.0, 0.0]"
# sleep 15
# rosservice call /uav8/control_manager/goto "goal: [15.75, 56.08, 13.5, 0.0]"
# sleep 15
# rosservice call /uav8/control_manager/goto "goal: [9.79, 45.25, 13.5, 0.0]"
# sleep 15
