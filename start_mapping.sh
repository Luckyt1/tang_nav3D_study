#!/bin/bash
set -eo pipefail

SESSION="${SESSION:-indoor_slam}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="/opt/ros/humble/setup.bash"
workspace_setup="$workspace_dir/install/setup.bash"
odometry_script="$workspace_dir/start_odometry.sh"
save_script="$workspace_dir/save_map.sh"

# 1. 选择模式；在检查完成前，不关闭正在运行的会话。
echo "请选择运行模式："
echo "  1) 建图（Odin 里程计 + FreeDOM 建图）"
echo "  2) 重定位（Odin 里程计 + BTC 重定位）"
echo "  3) 导航预览（重定位 + ScanPlanner + 全局路径，不输出速度）"
read -r -p "输入 1、2 或 3 [默认 1]：" mode || exit 1
case "${mode:-1}" in
    1)
        processing_package="map"
        processing_executable="map_node"
        processing_title="FreeDOM 建图"
        window_name="mapping"
        rviz_config="$workspace_dir/config/mapping.rviz"
        ;;
    2)
        processing_package="relocalization"
        processing_executable="relocalization_node"
        processing_title="BTC 重定位"
        window_name="relocalization"
        rviz_config="$workspace_dir/config/relocalization.rviz"
        ;;
    3)
        processing_package="planner"
        processing_executable="scan_planner_node"
        processing_title="BTC 定位 + ScanPlanner 导航预览"
        window_name="navigation"
        rviz_config="$workspace_dir/src/planner/config/navigation.rviz"
        ;;
    *)
        echo "错误：请输入 1、2 或 3。" >&2
        exit 1
        ;;
esac

is_btc_snapshot() {
    local directory="$1" descriptor
    [[ -f "$directory/metadata.yaml" && -f "$directory/poses.csv" &&
       -f "$directory/static_map.pcd" &&
       -f "$directory/btc/manifest.csv" && ! -e "$directory/.incomplete" ]] || return 1
    for descriptor in "$directory"/btc/*.btc; do
        [[ -f "$descriptor" ]] && return 0
    done
    return 1
}

if [[ "$processing_package" != "map" ]]; then
    map_directory="${MAP_DIR:-}"
    if [[ -z "$map_directory" ]]; then
        map_directories=()
        for candidate in "$workspace_dir"/maps/*; do
            if is_btc_snapshot "$candidate"; then
                map_directories+=("$candidate")
            fi
        done
        if [[ ${#map_directories[@]} -eq 0 ]]; then
            echo "错误：maps/ 下没有含 BTC 描述子的已完成地图。" >&2
            echo "请先选 1 建图，用当前 map 节点调用 /map/save_map 保存。" >&2
            echo "其他位置的地图可用 MAP_DIR='/完整地图目录' ./start_mapping.sh 指定。" >&2
            exit 1
        fi

        # 默认选择目录名排序最后的一份；本工程保存目录以时间戳命名。
        map_choice="${#map_directories[@]}"
        if [[ ${#map_directories[@]} -gt 1 ]]; then
            echo "可用地图："
            for index in "${!map_directories[@]}"; do
                printf '  %d) %s\n' "$((index + 1))" "${map_directories[index]##*/}"
            done
            read -r -p "选择地图编号 [默认 $map_choice]：" selection || exit 1
            map_choice="${selection:-$map_choice}"
        fi
        for index in "${!map_directories[@]}"; do
            if [[ "$map_choice" == "$((index + 1))" ]]; then
                map_directory="${map_directories[index]}"
                break
            fi
        done
    fi
    if [[ -z "$map_directory" ]] || ! is_btc_snapshot "$map_directory"; then
        echo "错误：地图编号无效，或地图缺少 BTC 文件/尚未保存完成。" >&2
        exit 1
    fi
    map_directory="$(cd -- "$map_directory" && pwd)"
    echo "重定位地图：$map_directory"
fi

# 2. 检查运行环境和所选模式的节点。
required_files=("$ros_setup" "$workspace_setup" "$odometry_script" "$rviz_config")
if [[ "$processing_package" == "map" ]]; then
    required_files+=("$save_script")
fi
for required_file in "${required_files[@]}"; do
    if [[ ! -f "$required_file" ]]; then
        echo "错误：缺少 $required_file" >&2
        echo "请确认 ROS 2 Humble 已安装，并在工作区执行：" >&2
        echo "  colcon build --packages-up-to odin_ros_driver $processing_package" >&2
        exit 1
    fi
done

# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$workspace_setup"
export ROS_DOMAIN_ID

required_nodes=("odin_ros_driver host_sdk_sample" "$processing_package $processing_executable" "rviz2 rviz2")
if [[ "$processing_package" == "planner" ]]; then
    required_nodes+=("relocalization relocalization_node" "planner octo_global_planner_node")
fi
for node in "${required_nodes[@]}"; do
    read -r package executable <<< "$node"
    if ! package_prefix="$(ros2 pkg prefix "$package")" ||
       [[ ! -x "$package_prefix/lib/$package/$executable" ]]; then
        echo "错误：找不到可执行节点 $package/$executable，请先编译对应包。" >&2
        exit 1
    fi
done

# 3. 检查并安装 tmux。
if ! command -v tmux &> /dev/null; then
    sudo apt-get update
    sudo apt-get install -y tmux
fi

# 4. 清理同名旧会话后重新创建，避免重复启动节点。
# = 表示精确匹配会话名，避免匹配到名称相近的其他会话。
tmux kill-session -t "=$SESSION" 2>/dev/null || true
odom_pane="$(tmux new-session -d -s "$SESSION" -n "$window_name" \
    -c "$workspace_dir" -P -F '#{pane_id}' 'bash --noprofile --norc')"

# 5. 左侧里程计，右侧运行所选模式；建图时再划出右下保存窗格。
processing_pane="$(tmux split-window -h -p 50 -t "$odom_pane" \
    -c "$workspace_dir" -P -F '#{pane_id}' 'bash --noprofile --norc')"
if [[ "$processing_package" == "map" ]]; then
    save_pane="$(tmux split-window -v -p 30 -t "$processing_pane" \
        -c "$workspace_dir" -P -F '#{pane_id}' 'bash --noprofile --norc')"
    tmux select-pane -t "$save_pane" -T "地图保存（s + 回车）"
fi

tmux set-option -t "$SESSION" mouse on
tmux set-option -w -t "$SESSION:$window_name" pane-border-status top
tmux set-option -w -t "$SESSION:$window_name" pane-border-format \
    ' #[fg=black,bg=green] #T #[default] '
tmux select-pane -t "$odom_pane" -T "Odin 里程计"
tmux select-pane -t "$processing_pane" -T "$processing_title"

# RViz 单独放在后台窗口，日志不占用建图/重定位布局；停止会话时一并关闭。
rviz_pane="$(tmux new-window -d -t "$SESSION:" -n rviz \
    -c "$workspace_dir" -P -F '#{pane_id}' 'bash --noprofile --norc')"
tmux select-pane -t "$rviz_pane" -T "RViz"

# 6. 组装命令。%q 会正确处理工作区和地图路径中的空格。
printf -v odom_command \
    'cd %q && export ROS_DOMAIN_ID=%q && source %q && bash %q' \
    "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$odometry_script"
printf -v processing_command \
    'cd %q && export ROS_DOMAIN_ID=%q && source %q && source %q && ros2 run %q %q' \
    "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$workspace_setup" \
    "$processing_package" "$processing_executable"
if [[ "$processing_package" == "relocalization" ]]; then
    printf -v map_argument ' --ros-args -p %q' "map.directory:=$map_directory"
    processing_command+="$map_argument"
elif [[ "$processing_package" == "planner" ]]; then
    printf -v processing_command \
        'cd %q && export ROS_DOMAIN_ID=%q && source %q && source %q && ros2 launch planner navigation.launch.py %q start_rviz:=false start_controller:=false %q %q %q %q %q %q' \
        "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$workspace_setup" \
        "map_directory:=$map_directory" "robot_radius:=${ROBOT_RADIUS:-0.3}" \
        "body_height:=${BODY_HEIGHT:-0.35}" "ground_height:=${GROUND_HEIGHT:--1.1}" \
        "mount_roll:=${MOUNT_ROLL:-0.0}" "mount_pitch:=${MOUNT_PITCH:-0.7853981633974483}" \
        "mount_yaw:=${MOUNT_YAW:-0.0}"
fi
printf -v rviz_command \
    'cd %q && export ROS_DOMAIN_ID=%q && source %q && source %q && ros2 run rviz2 rviz2 -d %q' \
    "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$workspace_setup" "$rviz_config"

# 里程计脚本会生成 custom_map_mode=0 的临时配置，并发布点云、里程计和 TF。
tmux send-keys -t "$odom_pane" -l "$odom_command"
tmux send-keys -t "$odom_pane" C-m
sleep 2

tmux send-keys -t "$processing_pane" -l "$processing_command"
tmux send-keys -t "$processing_pane" C-m
tmux send-keys -t "$rviz_pane" -l "$rviz_command"
tmux send-keys -t "$rviz_pane" C-m

echo "已启动：$SESSION，ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
if [[ "$processing_package" == "map" ]]; then
    printf -v save_command 'cd %q && export ROS_DOMAIN_ID=%q && bash %q' \
        "$workspace_dir" "$ROS_DOMAIN_ID" "$save_script"
    tmux send-keys -t "$save_pane" -l "$save_command"
    tmux send-keys -t "$save_pane" C-m
    tmux select-pane -t "$save_pane"
    echo "左侧：Odin 里程计；右上：FreeDOM 建图；右下：输入 s 并回车保存地图。"
else
    tmux select-pane -t "$odom_pane"
    echo "左侧：Odin 里程计；右侧：$processing_title。"
fi
echo "RViz 已随所选模式启动；日志位于 tmux 的 rviz 窗口（Ctrl+B 后按 W 切换）。"
if [[ "$processing_package" == "planner" ]]; then
    echo "定位有效后，在 RViz 用 2D Goal Pose 选择目标；显示全局路径和局部轨迹。"
    echo "沿用 3D_nav 中已调好的机器人参数，安装俯仰角为 45°；支持环境变量或 launch 参数覆盖。"
fi
echo "Ctrl+B 后按 D 可退出界面并保留运行。"
echo "停止节点和 RViz：./stop_mapping.sh"

if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach-session -t "$SESSION"
fi
