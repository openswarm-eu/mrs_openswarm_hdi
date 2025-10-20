#!/usr/bin/env bash
set -euo pipefail

# Name of tmux session
SESSION="drones"

# UAV hosts and their IPs (fill uav15 if you have it)
declare -A uavs=(
  [uav6_vpn]=192.168.194.170
  [uav7_vpn]=192.168.194.188
  [uav8_vpn]=192.168.194.254
  [uav9_vpn]=192.168.194.228
  [uav10_vpn]=192.168.194.190
  [uav11_vpn]=192.168.194.160
  [uav12_vpn]=192.168.194.137
  [uav13_vpn]=192.168.194.144
  [uav14_vpn]=192.168.194.203
  [uav15_vpn]=""   # <-- Replace "" with the IP for uav15, if available
)

# Ordered list of UAVs (10 drones → 20 panes: ssh + ping)
uav_list=(uav6_vpn uav7_vpn uav8_vpn uav9_vpn uav10_vpn uav11_vpn uav12_vpn uav13_vpn uav14_vpn uav15_vpn)

# If session exists, kill it to start fresh
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

# Start new detached session
tmux new-session -d -s "$SESSION" -n main

# Step 1: create 5 rows (single column)
# We repeatedly split the top pane vertically so the top pane remains the reference,
# producing 5 stacked rows from top -> bottom.
for i in $(seq 1 4); do
  tmux select-pane -t "$SESSION":0.0
  tmux split-window -v -t "$SESSION":0.0
done

# Step 2: collect the 5 row-top pane IDs sorted by their top coordinate (top->bottom).
mapfile -t row_panes < <(
  tmux list-panes -t "$SESSION":0 -F "#{pane_id} #{pane_top}" \
    | sort -n -k2 \
    | awk '{print $1}'
)

# sanity check
if [ "${#row_panes[@]}" -ne 5 ]; then
  echo "ERROR: expected 5 row panes but found ${#row_panes[@]}." >&2
  tmux attach-session -t "$SESSION"
  exit 1
fi

# Step 3: for each row-pane, split horizontally 3 times to get 4 columns per row
# We target the original left pane of the row each time so splits happen left->right.
for pane in "${row_panes[@]}"; do
  # create 3 horizontal splits -> total 4 columns in this row
  tmux split-window -h -t "$pane"
  tmux split-window -h -t "$pane"
  tmux split-window -h -t "$pane"
done

# Step 4: produce a display-ordered list of all panes sorted by row (top) then column (left)
mapfile -t panes_order < <(
  tmux list-panes -t "$SESSION":0 -F "#{pane_id} #{pane_top} #{pane_left}" \
    | sort -n -k2 -k3 \
    | awk '{print $1}'
)

# sanity check: must be exactly 20 panes
if [ "${#panes_order[@]}" -ne 20 ]; then
  echo "ERROR: expected 20 panes but found ${#panes_order[@]}. Aborting attach." >&2
  tmux list-panes -t "$SESSION":0 -F "#{pane_index} #{pane_id} #{pane_top} #{pane_left}"
  tmux attach-session -t "$SESSION"
  exit 1
fi

# Step 5: Send ssh / ping commands into panes in pairs (SSH then ping)
idx=0
for uav in "${uav_list[@]}"; do
  pane_ssh="${panes_order[$idx]}"
  pane_ping="${panes_order[$((idx+1))]}"

  # Send SSH command to first pane of pair
  # Keep it interactive (do not exec), so user can log in; user may have ssh config or keys.
  tmux send-keys -t "$pane_ssh" "ssh $uav" C-m
  tmux send-keys -t "$pane_ssh" "cd singularity_poc2/" C-m
  tmux send-keys -t "$pane_ssh" "./wrapper.sh" C-m
  tmux send-keys -t "$pane_ssh" "clear" C-m

  # Send ping (color_ping.sh) or a placeholder if IP missing
  ip="${uavs[$uav]:-}"
  if [ -n "$ip" ]; then
    tmux send-keys -t "$pane_ping" "./color_ping.sh $ip" C-m
  else
    tmux send-keys -t "$pane_ping" "echo 'No IP configured for $uav'; bash" C-m
  fi

  tmux select-pane -t "$pane_ssh" -T "${uav}-ssh"
  tmux select-pane -t "$pane_ping" -T "${uav}-ping"

  idx=$((idx+2))
done

# Tidy layout (should already be grid)
tmux select-layout -t "$SESSION":0 tiled

# Attach to session
tmux attach-session -t "$SESSION"
