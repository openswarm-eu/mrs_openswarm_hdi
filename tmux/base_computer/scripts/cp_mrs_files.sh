#!/usr/bin/env bash

# === CONFIGURATION ===
# Path to updated folders (local)
FILES_TO_DEPLOY=("/home/ingeniarius/singularity_poc2/user_ros_workspace/src/mrs_openswarm_real/tmux/forest/")

# Remote directory where files are deployed
REMOTE_DIR="/singularity_poc2/user_ros_workspace/src/mrs_openswarm_real/tmux/"

usage() {
    echo "Usage: $0 [--plain|--vpn] [host1 host2 ...]"
    echo "  --plain  default hosts: uav6 ... uav15"
    echo "  --vpn    default hosts: uav6_vpn ... uav15_vpn"
}

build_default_hosts() {
    local format="$1"
    local i
    drone_hosts=()

    # Use seq instead of brace expansion to avoid dependence on shell option "braceexpand".
    for i in $(seq 7 15); do
        if [ "$format" = "vpn" ]; then
            drone_hosts+=("uav${i}_vpn")
        else
            drone_hosts+=("uav${i}")
        fi
    done
}

# === DEPLOY FUNCTION ===
deploy_files() {
    local host="$1"
    local remote_user=""

    if [[ "$host" =~ ^uav([0-9]+)(_vpn)?$ ]]; then
        remote_user="uav_${BASH_REMATCH[1]}"
    else
        echo "Failed: unsupported host format '$host' (expected uav<id> or uav<id>_vpn)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    fi

    echo "Deploying to $host."

    for folder in "${FILES_TO_DEPLOY[@]}"; do
        if ! scp -rq "$folder" "$remote_user@$host:/home/${remote_user}${REMOTE_DIR}/"; then
            echo "❌ Failed to copy $folder to $host"
            FAIL_COUNT=$((FAIL_COUNT + 1))
        else
            echo "✅ $folder copied to $host"
            SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
        fi
    done
}

# === MAIN LOOP ===
echo "Starting..."

FAIL_COUNT=0
SUCCESS_COUNT=0

host_format="vpn"
while [ $# -gt 0 ]; do
    case "$1" in
        --plain)
            host_format="plain"
            shift
            ;;
        --vpn)
            host_format="vpn"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

if [ $# -gt 0 ]; then
    drone_hosts=("$@")
else
    build_default_hosts "$host_format"
fi

echo "Selected hosts (${#drone_hosts[@]}): ${drone_hosts[*]}"

for host in "${drone_hosts[@]}"; do
    deploy_files "$host" || true
done

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo "Process completed with errors: $FAIL_COUNT failed, $SUCCESS_COUNT successful across ${#drone_hosts[@]} hosts."
    exit 1
fi

echo "Process completed successfully: $SUCCESS_COUNT successful across ${#drone_hosts[@]} hosts."
exit 0
