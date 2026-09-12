#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SETUP_FILE="$WORKSPACE_ROOT/install/setup.bash"
CONFIG_FILE="$SCRIPT_DIR/config/control_command.yaml"
LOG_FILE="$SCRIPT_DIR/log/start_$(date +%Y%m%d_%H%M%S).log"
DRIVER_PID=""

if [ ! -f "$SETUP_FILE" ]; then
    echo "错误：未找到 $SETUP_FILE"
    echo "请先执行：$SCRIPT_DIR/script/build_ros2.sh"
    exit 1
fi

# shellcheck disable=SC1090
source "$SETUP_FILE"

start_mapping() {
    if [ -n "$DRIVER_PID" ] && kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "Odin 建图已在运行，PID=$DRIVER_PID"
        return
    fi

    if ! grep -Eq '^[[:space:]]*custom_map_mode:[[:space:]]*1([[:space:]#]|$)' "$CONFIG_FILE"; then
        echo "错误：$CONFIG_FILE 中 custom_map_mode 必须为 1 才能建图并保存。"
        return
    fi

    mkdir -p "$(dirname "$LOG_FILE")"
    ros2 launch odin_ros_driver odin1_ros2.launch.py \
        "config_file:=$CONFIG_FILE" >"$LOG_FILE" 2>&1 &
    DRIVER_PID=$!
    sleep 2

    if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "启动失败，日志如下："
        tail -n 30 "$LOG_FILE"
        DRIVER_PID=""
        return
    fi

    echo "已启动 Odin SLAM 建图，PID=$DRIVER_PID"
    echo "运行日志：$LOG_FILE"
}

save_map() {
    if [ -z "$DRIVER_PID" ] || ! kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "驱动未运行，请先输入 1 启动建图。"
        return
    fi

    if ! ros2 service list 2>/dev/null | grep -Fxq '/odin1/save_map'; then
        echo "保存服务尚未就绪，请等待设备连接后重试。"
        return
    fi

    if ros2 service call /odin1/save_map odin_ros_driver/srv/SaveMap '{value: 1}'; then
        echo "保存请求已发送；地图传输在后台进行，请至少等待 5 秒后再保存或退出。"
        echo "默认地图目录：$SCRIPT_DIR/map/"
        echo "保存进度：tail -f $LOG_FILE"
    else
        echo "保存命令执行失败，请查看日志：$LOG_FILE"
    fi
}

stop_mapping() {
    if [ -n "$DRIVER_PID" ] && kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "正在停止 Odin 建图，PID=$DRIVER_PID"
        kill -INT "$DRIVER_PID"
        wait "$DRIVER_PID" 2>/dev/null || true
    fi
    DRIVER_PID=""
}

trap stop_mapping EXIT
trap 'exit 130' INT TERM

while true; do
    echo
    echo "===== Odin ROS Driver ====="
    echo "1) 启动建图"
    echo "2) 保存地图"
    echo "3) 停止并退出（保存过程中不要执行）"
    read -r -p "请输入 1/2/3: " choice || exit 0

    case "$choice" in
        1) start_mapping ;;
        2) save_map ;;
        3) exit 0 ;;
        *) echo "无效输入，请输入 1、2 或 3。" ;;
    esac
done
