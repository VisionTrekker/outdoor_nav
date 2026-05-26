#!/usr/bin/env bash
# benchmark_mavros.sh — 采样 mavros_node 的 CPU/内存并统计话题数
# 用法: ./benchmark_mavros.sh [采样秒数，默认60]
#
# 前提: mavros_node 已在另一个终端启动并稳定运行
# 依赖: pidstat (sysstat 包), ros2 环境已 source

set -eo pipefail

DURATION=${1:-60}
SAMPLE_INTERVAL=2

echo "================================================"
echo "  MAVROS CPU Benchmark  (采样 ${DURATION}s)"
echo "================================================"

# 等 mavros 完全启动
echo ""
echo "[0] 等待 mavros_node 就绪..."
for i in $(seq 1 30); do
    MAVROS_PID=$(pgrep -f "mavros_node" | head -1)
    if [ -n "$MAVROS_PID" ]; then
        echo "    mavros_node PID = $MAVROS_PID"
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo "    错误: mavros_node 未运行!"
        exit 1
    fi
    sleep 1
done

# 等待 10s 让 mavros 完成插件加载和节点初始化
echo "[0] 等待 10s 让 mavros 完成初始化..."
sleep 10

# ─── 方法1: pidstat (最精确) ────────────────────────────
echo ""
echo "================================================"
echo "[1] pidstat 采样 (每 ${SAMPLE_INTERVAL}s 一次，共 ${DURATION}s)"
echo "================================================"
if command -v pidstat &>/dev/null; then
    pidstat -p "$MAVROS_PID" "$SAMPLE_INTERVAL" "$((DURATION / SAMPLE_INTERVAL))"
    echo ""
    echo ">>> pidstat 平均 CPU:"
    pidstat -p "$MAVROS_PID" "$DURATION" 1 | awk '/Average/{print "    %usr="$4" %sys="$5" %CPU="$8}'
else
    echo "    [跳过] pidstat 未安装，请安装: sudo apt install sysstat"
fi

# ─── 话题数量 ─────────────────────────────────────────────
echo ""
echo "================================================"
echo "[2] MAVROS 话题统计"
echo "================================================"
TOTAL_TOPICS=$(ros2 topic list 2>/dev/null | grep -c /mavros/ || echo "N/A")
echo "    /mavros/* 话题总数: $TOTAL_TOPICS"

echo ""
echo "================================================"
echo "  Benchmark 完成"
echo "================================================"
