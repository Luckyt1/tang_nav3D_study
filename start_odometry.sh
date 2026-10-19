#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpu_launcher="$workspace_dir/scripts/run_on_nav_cpus.py"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
setup_file="$workspace_dir/install/setup.bash"
base_config="$workspace_dir/src/odin_ros_driver/config/control_command.yaml"
sensor_backend="${SENSOR_BACKEND:-odin}"

if [[ "$sensor_backend" != "odin" && "$sensor_backend" != "mid360" ]]; then
  echo "错误：SENSOR_BACKEND 必须为 odin 或 mid360。" >&2
  exit 1
fi
for required_file in "$ros_setup" "$setup_file"; do
  if [[ ! -f "$required_file" ]]; then
    echo "错误：未找到 $required_file，请先安装 ROS 2 Humble 并编译工作区。" >&2
    exit 1
  fi
done

# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$setup_file"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"

if [[ "$sensor_backend" == "mid360" ]]; then
  point_lio_config="${POINT_LIO_CONFIG:-$workspace_dir/src/Point-LIO/config/mid360.yaml}"
  livox_config="${LIVOX_CONFIG:-$workspace_dir/src/livox_ros_driver2/config/MID360s_config.json}"
  for config_file in "$point_lio_config" "$livox_config"; do
    if [[ ! -f "$config_file" ]]; then
      echo "错误：未找到配置 $config_file。" >&2
      exit 1
    fi
  done
  point_lio_config="$(realpath -e -- "$point_lio_config")"
  livox_config="$(realpath -e -- "$livox_config")"
  for node in "livox_ros_driver2 livox_ros_driver2_node" "point_lio pointlio_mapping"; do
    read -r package executable <<< "$node"
    if ! prefix="$(ros2 pkg prefix "$package")" || [[ ! -x "$prefix/lib/$package/$executable" ]]; then
      echo "错误：缺少 $package/$executable，请先 colcon build --packages-up-to point_lio planner。" >&2
      exit 1
    fi
  done
  if ! prefix="$(ros2 pkg prefix planner)" ||
     [[ ! -f "$prefix/share/planner/launch/mid360_odometry.launch.py" ]]; then
    echo "错误：缺少 MID360S 启动文件，请先编译 planner。" >&2
    exit 1
  fi
  echo "正在启动 MID360S + Point-LIO，按 Ctrl+C 停止……"
  exec python3 "$cpu_launcher" ros2 launch planner mid360_odometry.launch.py \
    "point_lio_config:=$point_lio_config" "livox_config:=$livox_config"
fi

if ! prefix="$(ros2 pkg prefix odin_ros_driver)" ||
   [[ ! -x "$prefix/lib/odin_ros_driver/host_sdk_sample" ]] || [[ ! -f "$base_config" ]]; then
  echo "错误：缺少 Odin 驱动或配置，请先编译 odin_ros_driver。" >&2
  exit 1
fi

runtime_config="$(mktemp /tmp/odin_odometry_XXXXXX.yaml)"
trap 'rm -f "$runtime_config"' EXIT

sed -E \
  -e 's/^([[:space:]]*custom_map_mode:).*/\1 0      # 纯里程计模式/' \
  -e 's|^([[:space:]]*relocalization_map_abs_path:).*|\1 ""|' \
  "$base_config" > "$runtime_config"

if ! grep -Eq '^[[:space:]]*custom_map_mode:[[:space:]]*0([[:space:]#]|$)' "$runtime_config"; then
  echo "错误：无法生成纯里程计模式配置。" >&2
  exit 1
fi

echo "正在启动 Odin 里程计，按 Ctrl+C 停止……"
python3 "$cpu_launcher" ros2 run odin_ros_driver host_sdk_sample --ros-args -p "config_file:=$runtime_config"
