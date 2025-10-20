#!/bin/bash

# List of 9 hosts to ping
# uav6_vpn: 192.168.194.170
# uav7_vpn: 192.168.194.188
# uav8_vpn: 192.168.194.254
# uav9_vpn: 192.168.194.228
# uav10_vpn: 192.168.194.190
# uav11_vpn: 192.168.194.160
# uav12_vpn: 192.168.194.137
# uav13_vpn: 192.168.194.144
# uav14_vpn: 192.168.194.203
# uav15_vpn: 

# List of 9 hosts to ping
hosts=(
    192.168.8.170
    192.168.8.190
    192.168.8.160
    192.168.8.188
    192.168.8.254
    192.168.8.228
    192.168.8.137
    192.168.8.144
    192.168.8.146
)

# Create a new tmux session named "ping"
tmux new-session -s ping -d

# First row
tmux send-keys "./color_ping.sh ${hosts[0]}" C-m
tmux split-window -h
tmux send-keys "./color_ping.sh ${hosts[1]}" C-m
tmux split-window -h
tmux send-keys "./color_ping.sh ${hosts[2]}" C-m

# Second row
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "./color_ping.sh ${hosts[3]}" C-m
tmux select-pane -t 1
tmux split-window -v
tmux send-keys "./color_ping.sh ${hosts[4]}" C-m
tmux select-pane -t 2
tmux split-window -v
tmux send-keys "./color_ping.sh ${hosts[5]}" C-m

# Third row
tmux select-pane -t 5
tmux split-window -v
tmux send-keys "./color_ping.sh ${hosts[6]}" C-m
tmux select-pane -t 6
tmux split-window -v
tmux send-keys "./color_ping.sh ${hosts[7]}" C-m
tmux select-pane -t 7
tmux split-window -v
tmux send-keys "./color_ping.sh ${hosts[8]}" C-m

# Arrange layout and attach
tmux select-layout tiled
tmux attach-session -t ping
