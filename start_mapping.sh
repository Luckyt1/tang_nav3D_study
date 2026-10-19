#!/bin/bash
set -eo pipefail

SESSION="${SESSION:-indoor_slam}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
START_RVIZ="${START_RVIZ:-1}"
case "$START_RVIZ" in
    0|1) ;;
    *) echo "错误：START_RVIZ 必须为 0（关闭）或 1（开启）。" >&2; exit 1 ;;
esac
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cpu_launcher="$workspace_dir/scripts/run_on_nav_cpus.py"
# Resolve before touching an existing session; tmux panes receive this exact set.
NAV_CPUSET="$(python3 "$cpu_launcher" --print-cpus)"
export NAV_CPUSET
echo "导航进程共用逻辑 CPU：$NAV_CPUSET"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
workspace_setup="$workspace_dir/install/setup.bash"
odometry_script="$workspace_dir/start_odometry.sh"
save_script="$workspace_dir/save_map.sh"

# 1. 选择模式；在检查完成前，不关闭正在运行的会话。
echo "请选择运行模式："
echo "  1) 建图（传感器里程计 + FreeDOM 建图）"
echo "  2) 重定位（传感器里程计 + BTC/ICP 重定位）"
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

# 里程计来源独立于工作模式；环境变量可跳过此菜单。
sensor_backend="${SENSOR_BACKEND:-}"
if [[ -z "$sensor_backend" ]]; then
    echo "请选择传感器："
    echo "  1) Odin（默认）"
    echo "  2) MID360S + Point-LIO"
    read -r -p "输入 1 或 2 [默认 1]：" sensor_choice || exit 1
    case "${sensor_choice:-1}" in
        1) sensor_backend="odin" ;;
        2) sensor_backend="mid360" ;;
        *) echo "错误：请输入 1 或 2。" >&2; exit 1 ;;
    esac
fi
case "$sensor_backend" in
    odin)
        odometry_title="Odin 里程计"
        input_topic="/odin1/cloud_slam"
        sensor_frame="lidar"
        default_roll="0.0"
        default_pitch="0.7853981633974483"
        ;;
    mid360)
        odometry_title="MID360S + Point-LIO"
        input_topic="/cloud_registered"
        sensor_frame="livox_frame"
        default_roll="3.141592653589793"
        default_pitch="0.0"
        ;;
    *) echo "错误：SENSOR_BACKEND 必须为 odin 或 mid360。" >&2; exit 1 ;;
esac
point_lio_config="${POINT_LIO_CONFIG:-$workspace_dir/src/Point-LIO/config/mid360.yaml}"
livox_config="${LIVOX_CONFIG:-$workspace_dir/src/livox_ros_driver2/config/MID360s_config.json}"

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
required_files=("$ros_setup" "$workspace_setup" "$odometry_script")
if [[ "$START_RVIZ" == "1" ]]; then
    required_files+=("$rviz_config")
fi
if [[ "$sensor_backend" == "mid360" ]]; then
    required_files+=("$point_lio_config" "$livox_config")
else
    required_files+=("$workspace_dir/src/odin_ros_driver/config/control_command.yaml")
fi
if [[ "$processing_package" == "map" ]]; then
    required_files+=("$save_script")
fi
for required_file in "${required_files[@]}"; do
    if [[ ! -f "$required_file" ]]; then
        echo "错误：缺少 $required_file" >&2
        echo "请确认 ROS 2 Humble 已安装，并在工作区执行：" >&2
        if [[ "$sensor_backend" == "mid360" ]]; then
            echo "  colcon build --packages-up-to point_lio planner" >&2
        else
            echo "  colcon build --packages-up-to odin_ros_driver $processing_package" >&2
        fi
        exit 1
    fi
done

# pane 会切换工作目录，先将调用者传入的相对路径解析为绝对路径。
ros_setup="$(realpath -e -- "$ros_setup")"
if [[ "$sensor_backend" == "mid360" ]]; then
    point_lio_config="$(realpath -e -- "$point_lio_config")"
    livox_config="$(realpath -e -- "$livox_config")"
fi

# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$workspace_setup"
export ROS_DOMAIN_ID

required_nodes=("$processing_package $processing_executable")
if [[ "$START_RVIZ" == "1" ]]; then
    required_nodes+=("rviz2 rviz2")
fi
if [[ "$sensor_backend" == "mid360" ]]; then
    required_nodes+=("livox_ros_driver2 livox_ros_driver2_node" "point_lio pointlio_mapping")
    if ! planner_prefix="$(ros2 pkg prefix planner)" ||
       [[ ! -f "$planner_prefix/share/planner/launch/mid360_odometry.launch.py" ]]; then
        echo "错误：缺少 MID360S 启动文件，请先 colcon build --packages-up-to point_lio planner。" >&2
        exit 1
    fi
else
    required_nodes+=("odin_ros_driver host_sdk_sample")
fi
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
tmux select-pane -t "$odom_pane" -T "$odometry_title"
tmux select-pane -t "$processing_pane" -T "$processing_title"

# RViz 单独放在后台窗口，日志不占用建图/重定位布局；停止会话时一并关闭。
if [[ "$START_RVIZ" == "1" ]]; then
    rviz_pane="$(tmux new-window -d -t "$SESSION:" -n rviz \
        -c "$workspace_dir" -P -F '#{pane_id}' 'bash --noprofile --norc')"
    tmux select-pane -t "$rviz_pane" -T "RViz"
fi

# 6. 组装命令。%q 会正确处理工作区和地图路径中的空格。
printf -v odom_command \
    'cd %q && export ROS_DOMAIN_ID=%q SENSOR_BACKEND=%q POINT_LIO_CONFIG=%q LIVOX_CONFIG=%q ROS_SETUP=%q && source %q && bash %q' \
    "$workspace_dir" "$ROS_DOMAIN_ID" "$sensor_backend" "$point_lio_config" \
    "$livox_config" "$ros_setup" "$ros_setup" "$odometry_script"
printf -v processing_command \
    'cd %q && export ROS_DOMAIN_ID=%q && source %q && source %q && ros2 run %q %q' \
    "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$workspace_setup" \
    "$processing_package" "$processing_executable"
if [[ "$processing_package" != "planner" ]]; then
    printf -v sensor_arguments ' --ros-args -p %q -p %q' \
        "input_topic:=$input_topic" "sensor_frame:=$sensor_frame"
    processing_command+="$sensor_arguments"
fi
if [[ "$processing_package" == "relocalization" ]]; then
    printf -v map_argument ' -p %q' "map.directory:=$map_directory"
    processing_command+="$map_argument"
    if [[ "$sensor_backend" == "mid360" ]]; then
        # Point-LIO map and live odom both use gravity-aligned world axes.
        processing_command+=' -p fine.max_gravity_angle_deg:=30.0'
    fi
elif [[ "$processing_package" == "planner" ]]; then
    printf -v processing_command \
        'cd %q && export ROS_DOMAIN_ID=%q && source %q && source %q && ros2 launch planner navigation.launch.py %q start_rviz:=false start_controller:=false %q %q %q %q %q %q %q' \
        "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$workspace_setup" \
        "map_directory:=$map_directory" "sensor_backend:=$sensor_backend" "robot_radius:=${ROBOT_RADIUS:-0.3}" \
        "body_height:=${BODY_HEIGHT:-0.35}" "ground_height:=${GROUND_HEIGHT:--1.1}" \
        "mount_roll:=${MOUNT_ROLL:-$default_roll}" "mount_pitch:=${MOUNT_PITCH:-$default_pitch}" \
        "mount_yaw:=${MOUNT_YAW:-0.0}"
    if [[ "$sensor_backend" == "mid360" ]]; then
        printf -v config_argument ' %q' "point_lio_config:=$point_lio_config"
        processing_command+="$config_argument"
    fi
fi
if [[ "$START_RVIZ" == "1" ]]; then
    printf -v rviz_command \
        'cd %q && export ROS_DOMAIN_ID=%q && source %q && source %q && ros2 run rviz2 rviz2 -d %q' \
        "$workspace_dir" "$ROS_DOMAIN_ID" "$ros_setup" "$workspace_setup" "$rviz_config"
fi

# A pre-existing tmux server does not inherit the launching shell's affinity.
# Apply the same CPU set inside every pane, without changing the tmux server.
limit_cpu_command() {
    printf 'env NAV_CPUSET=%q python3 %q bash -c %q' \
        "$NAV_CPUSET" "$cpu_launcher" "$1"
}
odom_command="$(limit_cpu_command "$odom_command")"
processing_command="$(limit_cpu_command "$processing_command")"
if [[ "$START_RVIZ" == "1" ]]; then
    rviz_command="$(limit_cpu_command "$rviz_command")"
fi

# 选定的数据源发布 odom 系点云、里程计和 TF。
tmux send-keys -t "$odom_pane" -l "$odom_command"
tmux send-keys -t "$odom_pane" C-m
sleep 2

tmux send-keys -t "$processing_pane" -l "$processing_command"
tmux send-keys -t "$processing_pane" C-m
if [[ "$START_RVIZ" == "1" ]]; then
    tmux send-keys -t "$rviz_pane" -l "$rviz_command"
    tmux send-keys -t "$rviz_pane" C-m
fi

echo "已启动：$SESSION，ROS_DOMAIN_ID=$ROS_DOMAIN_ID，数据源=$sensor_backend"
if [[ "$processing_package" == "map" ]]; then
    printf -v save_command 'cd %q && export ROS_DOMAIN_ID=%q && bash %q' \
        "$workspace_dir" "$ROS_DOMAIN_ID" "$save_script"
    save_command="$(limit_cpu_command "$save_command")"
    tmux send-keys -t "$save_pane" -l "$save_command"
    tmux send-keys -t "$save_pane" C-m
    tmux select-pane -t "$save_pane"
    echo "左侧：$odometry_title；右上：FreeDOM 建图；右下：输入 s 并回车保存地图。"
else
    tmux select-pane -t "$odom_pane"
    echo "左侧：$odometry_title；右侧：$processing_title。"
fi
if [[ "$START_RVIZ" == "1" ]]; then
    echo "RViz 已启动（10 Hz，调试显示默认关闭）；日志位于 tmux 的 rviz 窗口。"
    echo "下次可用 START_RVIZ=0 ./start_mapping.sh 关闭 RViz，减少资源占用。"
else
    echo "RViz 已关闭（START_RVIZ=0），处理节点继续运行。"
fi
if [[ "$processing_package" == "planner" ]]; then
    if [[ "$START_RVIZ" == "1" ]]; then
        echo "定位有效后，在 RViz 用 2D Goal Pose 选择目标；显示全局路径和局部轨迹。"
    else
        echo "定位有效后，可从外部 RViz 的 2D Goal Pose 向 /goal_pose 发送目标。"
    fi
    echo "安装角：roll=${MOUNT_ROLL:-$default_roll}，pitch=${MOUNT_PITCH:-$default_pitch}，yaw=${MOUNT_YAW:-0.0}（弧度）。"
    echo "机器人尺寸、地面高度和安装角可用环境变量或 launch 参数覆盖。"
fi
echo "Ctrl+B 后按 D 可退出界面并保留运行。"
echo "停止节点和 RViz：./stop_mapping.sh"

if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach-session -t "$SESSION"
fi
