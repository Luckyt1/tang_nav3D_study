#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cpu_launcher="$workspace_dir/scripts/run_on_nav_cpus.py"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
for setup_file in "$ros_setup" "$workspace_dir/install/setup.bash"; do
  if [[ ! -f "$setup_file" ]]; then
    echo "错误：未找到 $setup_file，请先安装 ROS 并编译工作区。" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "$setup_file"
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_dir/log/odometry}"
mkdir -p -- "$ROS_LOG_DIR"

echo "启动 MID360S + Point-LIO 里程计和 RViz（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）。"
echo "启动时请保持雷达静止；RViz 固定坐标系为 odom，显示原始里程计和当前点云。"
echo "按 Ctrl+C 或关闭 RViz 窗口即可停止本次启动的全部节点。"

# Load the workspace launch directly so this viewer needs no planner rebuild.
exec python3 "$cpu_launcher" ros2 launch "$workspace_dir/src/planner/launch/mid360_odometry.launch.py" \
  "point_lio_config:=${POINT_LIO_CONFIG:-$workspace_dir/src/Point-LIO/config/mid360.yaml}" \
  "livox_config:=${LIVOX_CONFIG:-$workspace_dir/src/livox_ros_driver2/config/MID360s_config.json}" \
  start_rviz:=true \
  "rviz_config:=$workspace_dir/src/planner/config/odometry.rviz" "$@"
