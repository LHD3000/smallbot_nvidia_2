#!/usr/bin/env bash
# 快速启动 USB 相机 + ArUco 视觉检测
# 用法: ./start_vision.sh [video_device]

set -eo pipefail

WS_DIR="$HOME/jeston/smallbot_ws"
VIDEO_DEVICE="${1:-/dev/video0}"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

cleanup() {
    echo ""
    echo "正在停止相机节点..."
    if [[ -n "${CAM_PID:-}" ]] && kill -0 "$CAM_PID" 2>/dev/null; then
        kill "$CAM_PID" 2>/dev/null || true
        wait "$CAM_PID" 2>/dev/null || true
    fi
    echo "视觉检测已停止。"
}
trap cleanup EXIT INT TERM

echo "=== 启动 USB 相机: $VIDEO_DEVICE ==="
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="$VIDEO_DEVICE" \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=10.0 \
  -p pixel_format:="mjpeg2rgb" \
  -p camera_name:="usb_camera" \
  -p frame_id:="camera_link" &
CAM_PID=$!

sleep 2.0

echo "=== 启动 ArUco 检测 ==="
ros2 launch aruco aruco_det.launch.py
