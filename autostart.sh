#!/bin/bash
# =============================================================================
# autostart.sh — 自定义 ROS2 节点开机自启管理脚本
# 用法:
#   ./autostart.sh start    启动所有节点
#   ./autostart.sh stop     停止所有节点
#   ./autostart.sh restart  重启所有节点
#   ./autostart.sh status   查看节点状态
#   ./autostart.sh logs [executable]  查看日志（不指定则列出所有日志文件）
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF_FILE="$SCRIPT_DIR/autostart_nodes.conf"
PID_DIR="$SCRIPT_DIR/.autostart_pids"
LOG_DIR="$SCRIPT_DIR/logs"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"

WAIT_TOPIC_TIMEOUT=120   # 等待 topic 最长秒数
WAIT_TOPIC_INTERVAL=2    # 每次轮询间隔秒数

# ─────────────────────────────────────────────────────────────────────────────

_source_ros() {
    # shellcheck disable=SC1090
    source "$ROS_SETUP" 2>/dev/null || { echo "[ERROR] 找不到 $ROS_SETUP"; exit 1; }
    # shellcheck disable=SC1090
    source "$WS_SETUP"  2>/dev/null || { echo "[ERROR] 找不到 $WS_SETUP，请先 colcon build"; exit 1; }
}

_wait_topic() {
    local topic="$1"
    local elapsed=0
    echo "[INFO] 等待 topic: $topic"
    while ! ros2 topic info "$topic" > /dev/null 2>&1; do
        sleep "$WAIT_TOPIC_INTERVAL"
        elapsed=$((elapsed + WAIT_TOPIC_INTERVAL))
        if [ "$elapsed" -ge "$WAIT_TOPIC_TIMEOUT" ]; then
            echo "[WARN] 等待 $topic 超时（${WAIT_TOPIC_TIMEOUT}s），跳过等待继续启动"
            return 1
        fi
        echo "[INFO] 仍在等待 $topic ... (${elapsed}s)"
    done
    echo "[INFO] topic $topic 已就绪"
    return 0
}

_read_conf() {
    # 输出每行有效配置，格式: package executable [wait_topic]
    grep -v '^\s*#' "$CONF_FILE" | grep -v '^\s*$'
}

# ─────────────────────────────────────────────────────────────────────────────

cmd_start() {
    mkdir -p "$PID_DIR" "$LOG_DIR"
    _source_ros

    local started=0
    while IFS= read -r line; do
        read -r pkg exe wait_topic <<< "$line"
        [ -z "$pkg" ] || [ -z "$exe" ] && continue

        local pid_file="$PID_DIR/$exe.pid"
        if [ -f "$pid_file" ]; then
            local old_pid
            old_pid=$(cat "$pid_file")
            if kill -0 "$old_pid" 2>/dev/null || ps -p "$old_pid" > /dev/null 2>&1; then
                echo "[SKIP] $exe 已在运行 (PID $old_pid)"
                continue
            else
                rm -f "$pid_file"
            fi
        fi

        if [ -n "$wait_topic" ]; then
            _wait_topic "$wait_topic"
        fi

        local log_file="$LOG_DIR/$exe.log"
        echo "--- 启动于 $(date '+%Y-%m-%d %H:%M:%S') ---" >> "$log_file"

        # setsid 创建新进程组，stop 时可以连子进程一起杀
        setsid ros2 run "$pkg" "$exe" >> "$log_file" 2>&1 &
        local pid=$!
        echo "$pid" > "$pid_file"
        echo "[START] $pkg/$exe  PID=$pid  日志: logs/$exe.log"
        started=$((started + 1))
    done < <(_read_conf)

    [ "$started" -eq 0 ] && echo "[INFO] 没有新节点需要启动"
    exit 0
}

cmd_stop() {
    local stopped=0
    while IFS= read -r line; do
        read -r pkg exe _ <<< "$line"
        [ -z "$exe" ] && continue

        local pid_file="$PID_DIR/$exe.pid"
        if [ ! -f "$pid_file" ]; then
            echo "[SKIP] $exe 无 PID 记录"
            continue
        fi

        local pid
        pid=$(cat "$pid_file")

        # 先尝试杀掉整个进程组（setsid 创建的 session leader）
        if kill -- -"$pid" 2>/dev/null; then
            echo "[STOP] $exe  进程组 PGID=$pid"
            stopped=$((stopped + 1))
        elif kill -0 "$pid" 2>/dev/null; then
            # 回退：直接杀该 PID
            kill "$pid"
            echo "[STOP] $exe  PID=$pid"
            stopped=$((stopped + 1))
        else
            echo "[SKIP] $exe PID=$pid 已不存在"
        fi

        # 额外清理：按可执行文件名再杀一遍，确保 ros2 run 的子进程不留孤儿
        local actual_exe
        actual_exe=$(basename "$exe")
        if pgrep -x "$actual_exe" > /dev/null 2>&1; then
            pkill -9 -x "$actual_exe"
            echo "[CLEAN] $actual_exe 残留进程已清理"
        fi

        rm -f "$pid_file"
    done < <(_read_conf)

    [ "$stopped" -eq 0 ] && echo "[INFO] 没有正在运行的节点"
}

cmd_status() {
    echo "─────────────────────────────────────────────────────"
    printf "%-30s %-8s %s\n" "EXECUTABLE" "PID" "STATUS"
    echo "─────────────────────────────────────────────────────"

    while IFS= read -r line; do
        read -r pkg exe _ <<< "$line"
        [ -z "$exe" ] && continue

        local pid_file="$PID_DIR/$exe.pid"
        if [ -f "$pid_file" ]; then
            local pid
            pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                printf "%-30s %-8s \e[32m%s\e[0m\n" "$exe" "$pid" "running"
            else
                printf "%-30s %-8s \e[31m%s\e[0m\n" "$exe" "$pid" "dead (stale pid)"
            fi
        else
            printf "%-30s %-8s \e[33m%s\e[0m\n" "$exe" "-" "not started"
        fi
    done < <(_read_conf)

    echo "─────────────────────────────────────────────────────"
}

cmd_logs() {
    local target="$1"
    if [ -n "$target" ]; then
        local log_file="$LOG_DIR/$target.log"
        if [ -f "$log_file" ]; then
            tail -f "$log_file"
        else
            echo "[ERROR] 找不到日志: $log_file"
        fi
    else
        echo "可用日志:"
        ls "$LOG_DIR"/*.log 2>/dev/null | while read -r f; do
            echo "  $(basename "$f")"
        done
        echo ""
        echo "用法: $0 logs <executable>"
    fi
}

# ─────────────────────────────────────────────────────────────────────────────

case "${1:-}" in
    start)   cmd_start ;;
    stop)    cmd_stop ;;
    restart) cmd_stop; sleep 1; cmd_start ;;
    status)  cmd_status ;;
    logs)    cmd_logs "${2:-}" ;;
    *)
        echo "用法: $0 {start|stop|restart|status|logs [executable]}"
        echo "配置文件: $CONF_FILE"
        exit 1
        ;;
esac
