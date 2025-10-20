#!/bin/bash

rosservice call /$UAV_NAME/control_manager/goto "goal: [9.78, 45.25, 12.0, 0.0]"
sleep 15
rosservice call /$UAV_NAME/control_manager/goto "goal: [9.64, 54.55 , 12.0, 0.0]"
sleep 15
rosservice call /$UAV_NAME/control_manager/goto "goal: [2.00, 46.46, 12.0, 0.0]"
sleep 15