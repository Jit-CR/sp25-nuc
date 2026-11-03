#!/bin/bash
# ============================================
# sp_vision_25 一键启动 / 停止 / 查看 装甲板自瞄
# 用法:
#   ./run.sh          启动 (默认3号步兵)
#   ./run.sh 4        启动 4号步兵
#   ./run.sh stop     停止
#   ./run.sh status   查看运行状态
#   ./run.sh log      接入画面 (Ctrl+A 再按 D 退出)
# ============================================

# 切换到脚本所在目录，保证路径正确
cd "$(dirname "$0")" || exit 1
SESSION="aim"

# SSH 远程启动时，把画面输出到 NUC 本机显示器
export DISPLAY="${DISPLAY:-:0}"

# ---------- 停止 ----------
if [ "$1" = "stop" ]; then
    screen -X -S "$SESSION" quit
    echo "自瞄已停止"
    exit 0
fi

# ---------- 状态 ----------
if [ "$1" = "status" ]; then
    screen -ls
    exit 0
fi

# ---------- 接入画面 ----------
if [ "$1" = "log" ]; then
    screen -r "$SESSION"
    exit 0
fi

# ---------- 选择配置 ----------
case "$1" in
    4) CONFIG="configs/standard4.yaml" ;;
    *) CONFIG="configs/standard3.yaml" ;;
esac

# ---------- 没有可执行文件则先编译 ----------
if [ ! -f ./build/standard ]; then
    echo "未检测到 build/standard，开始编译..."
    cmake -B build || { echo "[错误] cmake 失败"; exit 1; }
    make -C build -j"$(nproc)" || { echo "[错误] 编译失败"; exit 1; }
fi

# ---------- 已在运行则先关闭，避免重复启动 ----------
if screen -ls | grep -q "[.]$SESSION"; then
    screen -X -S "$SESSION" quit
    sleep 1
fi

# ---------- 后台启动 ----------
screen -L -dmS "$SESSION" ./build/standard "$CONFIG"
sleep 1

echo "=========================================="
screen -ls
echo "自瞄已启动，配置文件: $CONFIG"
echo "查看画面: ./run.sh log    停止: ./run.sh stop"
echo "=========================================="
