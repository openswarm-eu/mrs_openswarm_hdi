# === CONFIGURATION ===
# Path to updated folders (local)
FILES_TO_DEPLOY=("/home/ingeniarius/singularity_poc2/user_ros_workspace/src/mrs_openswarm_real/tmux/forest/")

# Remote directory where files are deployed
REMOTE_DIR="/singularity_poc2/user_ros_workspace/src/mrs_openswarm_real/tmux/"

# === DEPLOY FUNCTION ===
deploy_files() {
    local original_host="$1"
    local host="${original_host}"

    local prefix="${original_host:0:3}"         # "uav"
    local number="${original_host:3}"           # "X"
    local remote_user="${prefix}_${number}"     # "uav_X"

    echo "Deploying to $host."

    for folder in "${FILES_TO_DEPLOY[@]}"; do
        folder_name=$(basename "$folder")

        scp -rq "$folder" "$remote_user@$host:/home/${remote_user}${REMOTE_DIR}/"
        if [ $? -ne 0 ]; then
            echo "❌ Failed to copy $folder to $host"
        else
            echo "✅ $folder copied to $host"
        fi
    done
}

# === MAIN LOOP ===
echo "Starting..."

if [ $# -gt 0 ]; then
    drone_hosts=("$1")   # Use the provided argument
else
    # drone_hosts=("uav6_vpn" "uav7_vpn" "uav8_vpn" "uav9_vpn" "uav10" "uav11" "uav12" "uav13" "uav14" "uav15")
    drone_hosts=("uav6" "uav7" "uav8" "uav9" "uav10" "uav11" "uav12" "uav13" "uav14" "uav15")
fi

for host in "${drone_hosts[@]}"; do
    deploy_files "$host"
done

echo "Process completed."
