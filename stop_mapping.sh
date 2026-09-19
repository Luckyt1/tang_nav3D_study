#!/bin/bash
set -eo pipefail

SESSION="${SESSION:-indoor_slam}"

if ! command -v tmux &> /dev/null; then
    echo "未安装 tmux，无需停止建图/重定位会话。"
    exit 0
fi

if ! tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "建图/重定位会话 $SESSION 未运行。"
    exit 0
fi

# 先向各窗格发送 Ctrl+C，让 ROS 节点退出并执行清理。
echo "正在停止 $SESSION 中的里程计、建图/重定位节点和 RViz……"
panes="$(tmux list-panes -s -t "=$SESSION" -F '#{pane_id}')"
while IFS= read -r pane; do
    # 在会话内部运行此脚本时，避免 Ctrl+C 中断停止脚本自身。
    if [[ "$pane" != "${TMUX_PANE:-}" ]]; then
        tmux send-keys -t "$pane" C-c 2>/dev/null || true
    fi
done <<< "$panes"
sleep 2

# 只关闭精确匹配的会话，保留其他 tmux 会话。
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    tmux kill-session -t "=$SESSION"
fi
echo "建图/重定位会话 $SESSION 及 RViz 已停止。"
