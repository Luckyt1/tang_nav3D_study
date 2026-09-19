#!/bin/bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="/opt/ros/humble/setup.bash"
workspace_setup="$workspace_dir/install/setup.bash"

for required_file in "$ros_setup" "$workspace_setup"; do
    if [[ ! -f "$required_file" ]]; then
        echo "错误：缺少 $required_file，请先安装 ROS 2 Humble 并编译工作区。" >&2
        exit 1
    fi
done

# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$workspace_setup"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"

echo "地图保存控制台（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）"
echo "输入 s 并回车：保存完整地图、子图、位姿和 BTC 描述子。"
echo "响应中的 success=True 表示保存成功，message 显示保存目录；失败时显示原因。"
echo "每次保存生成新目录。输入 q 并回车退出控制台，建图继续运行。"

while read -r -p "保存地图 [s] / 退出控制台 [q]：" action; do
    case "$action" in
        s|S)
            echo "正在请求 /map/save_map，请等待保存响应……"
            if ! ros2 service call /map/save_map std_srvs/srv/Trigger '{}'; then
                echo "保存服务调用失败，请检查建图节点后重试。" >&2
            fi
            ;;
        q|Q)
            break
            ;;
        "") ;;
        *) echo "请输入 s 保存，或 q 退出控制台。" ;;
    esac
done
