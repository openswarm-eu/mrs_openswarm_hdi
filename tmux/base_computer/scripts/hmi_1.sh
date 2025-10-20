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

# List of 3 hosts to ping
hosts=(
    192.168.194.188
)

# List of 9 hosts to ping
users=(
    uav7_vpn
)

# Create a new tmux session named "ping"
tmux new-session -s ping -d

# First row
tmux send-keys "ssh ${users[0]}" C-m
tmux split-window -h
tmux send-keys "./color_ping.sh ${hosts[0]}" C-m

# Arrange layout and attach
tmux select-layout tiled
tmux attach-session -t ping
