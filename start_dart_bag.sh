#!/bin/bash

set -e

WS=/home/pka/RM_PKA_2026_DartRack_AutoAim
BAG_ROOT=/home/pka/rm_bags
TIME_TAG=$(date +%Y%m%d_%H%M%S)
BAG_NAME=dart_traditional_${TIME_TAG}
BAG_PATH=${BAG_ROOT}/${BAG_NAME}

MAX_BAG_SIZE=2147483648   # 2GB 自动分包，单位是字节

echo "=========================================="
echo "Starting Dart Traditional ROS2 Bag Record"
echo "Workspace : ${WS}"
echo "Bag Path  : ${BAG_PATH}"
echo "=========================================="

source /opt/ros/humble/setup.bash

if [ ! -f "${WS}/install/setup.bash" ]; then
    echo "ERROR: 找不到 ${WS}/install/setup.bash"
    echo "请先确认工程已经 colcon build 成功。"
    exit 1
fi

source "${WS}/install/setup.bash"

mkdir -p "${BAG_ROOT}"

cd "${WS}"

echo "等待 /image_raw 话题出现..."

for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -Fxq "/image_raw"; then
        echo "检测到 /image_raw，开始录制。"
        break
    fi

    if [ "$i" -eq 30 ]; then
        echo "ERROR: 等待 30 秒后仍然没有发现 /image_raw。"
        echo "请确认相机节点和 dart.service 是否已经启动。"
        exit 1
    fi

    sleep 1
done

echo "即将录制以下话题："
echo "  /image_raw"
echo "  /result_img"
echo "  /light_position"
echo "  /filtered_x"
echo "  /filtered_y"
echo "  /yaw"
echo "  /serial_send_data"
echo "  /fire_state"
echo "  /camera_driver/heartbeat"
echo "  /dart_serial/heartbeat"
echo "=========================================="

exec ros2 bag record \
  /image_raw \
  /result_img \
  /light_position \
  /filtered_x \
  /filtered_y \
  /yaw \
  /serial_send_data \
  /fire_state \
  /camera_driver/heartbeat \
  /dart_serial/heartbeat \
  -o "${BAG_PATH}" \
  --max-bag-size "${MAX_BAG_SIZE}"