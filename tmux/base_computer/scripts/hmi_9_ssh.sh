#!/bin/bash
export TMUX_BIN="/usr/bin/tmux -L mrs -f /etc/ctu-mrs/tmux.conf"

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

# List of 9 hosts to ssh
hosts=(
    uav6_vpn
    uav10_vpn
    uav11_vpn
    uav7_vpn
    uav8_vpn
    uav9_vpn
    uav12_vpn
    uav13_vpn
    uav14_vpn
)

command="./singularity_poc2/wrapper.sh "

# Create a new tmux session named "ssh"
tmux new-session -s ssh -d

# Create the first row
tmux send-keys "ssh ${hosts[0]}" C-m
tmux split-window -h
tmux send-keys "ssh ${hosts[1]}" C-m
tmux split-window -h
tmux send-keys "ssh ${hosts[2]}" C-m

# Select the first pane to begin creating rows below
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "ssh ${hosts[3]}" C-m
tmux select-pane -t 1
tmux split-window -v
tmux send-keys "ssh ${hosts[4]}" C-m
tmux select-pane -t 2
tmux split-window -v
tmux send-keys "ssh ${hosts[5]}" C-m

# Create the third row
tmux select-pane -t 5
tmux split-window -v
tmux send-keys "ssh ${hosts[6]}" C-m
tmux select-pane -t 6
tmux split-window -v
tmux send-keys "ssh ${hosts[7]}" C-m
tmux select-pane -t 7
tmux split-window -v
tmux send-keys "ssh ${hosts[8]}" C-m

# Attach to the session
tmux select-layout tiled
# tmux set-window-option -t ssh synchronize-panes on
tmux attach-session -t ssh
