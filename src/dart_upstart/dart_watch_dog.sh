#!/bin/bash
# dart_watch_dog.sh
# 混合监控版：
# - dart_serial 使用 heartbeat 监控
# - dart_detector / dart_solver_node 使用 ros2 node list 监控
#
# 这样不需要修改 detector / solver 的 C++ 代码，也不会因为它们没有 heartbeat 而反复重启。

TIMEOUT=7

# launch_params.yaml 里 namespace: '' 时，这里保持空字符串。
# 如果以后改成 namespace: '/xxx'，这里也要写成 '/xxx'。
NAMESPACE=""

USER_NAME="$(whoami)"
HOME_DIR=$(eval echo ~$USER_NAME)

# 你的实际工作空间路径。
WORKING_DIR="$HOME_DIR/RM_PKA_2026_DartRack_AutoAim"

LAUNCH_FILE="dart_bringup bringup.launch.py"
OUTPUT_FILE="$WORKING_DIR/screen.output"

rmw="rmw_fastrtps_cpp"
export RMW_IMPLEMENTATION="$rmw"

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

# 没有 heartbeat 的节点，用 ros2 node list 检查是否存在。
NODE_ONLY_NODES=(
  "dart_detector"
  "dart_solver_node"
)

# 已经有 heartbeat 的节点，用 heartbeat 检查。
HEARTBEAT_NODES=(
  "dart_serial"

  # 如果相机节点也有 heartbeat，再取消对应注释。
  # 大华相机节点：
  "camera_driver"

  # 海康相机节点：
  # "hik_camera_driver"
)

source /opt/ros/humble/setup.bash
source "$WORKING_DIR/install/setup.bash" || {
  echo "ERROR: 无法加载工作空间，检查路径是否正确: $WORKING_DIR"
  exit 1
}

# RMW 配置，可按需填写配置文件路径。
rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" && -n "$rmw_config" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$rmw_config"
elif [[ "$rmw" == "rmw_cyclonedds_cpp" && -n "$rmw_config" ]]; then
  export CYCLONEDDS_URI="$rmw_config"
fi

function kill_nodes() {
  pkill -f "ros2 launch $LAUNCH_FILE" >/dev/null 2>&1

  for node in "${NODE_ONLY_NODES[@]}"; do
    pkill -f "$node" >/dev/null 2>&1
  done

  for node in "${HEARTBEAT_NODES[@]}"; do
    pkill -f "$node" >/dev/null 2>&1
  done
}

function bringup() {
  echo "[$(date)] 启动所有节点..."

  source /opt/ros/humble/setup.bash
  source "$WORKING_DIR/install/setup.bash"

  kill_nodes
  ros2 daemon stop >/dev/null 2>&1

  cd "$WORKING_DIR" || exit 1
  nohup ros2 launch $LAUNCH_FILE > "$OUTPUT_FILE" 2>&1 &

  sleep 15
}

function restart() {
  echo "[$(date)] 重启所有节点..."

  kill_nodes
  ros2 daemon stop >/dev/null 2>&1
  sleep 2

  bringup
}

# 初始启动节点
bringup

# 等待节点完成初始化
sleep $TIMEOUT
sleep $TIMEOUT

while true; do
  need_restart=0
  node_list=$(ros2 node list 2>/dev/null)

  # 1. 检查没有 heartbeat 的节点是否存在
  for node in "${NODE_ONLY_NODES[@]}"; do
    if [ -z "$NAMESPACE" ]; then
      full_node="/$node"
    else
      full_node="$NAMESPACE/$node"
    fi

    echo "[$(date)] 检查节点是否存在: $full_node"

    if echo "$node_list" | grep -q "^${full_node}$"; then
      echo "[$(date)]   $full_node 存在"
    else
      echo "[$(date)]   $full_node 不存在，重启中..."
      restart
      need_restart=1
      break
    fi
  done

  if [ "$need_restart" -eq 1 ]; then
    sleep $TIMEOUT
    continue
  fi

  # 2. 检查有 heartbeat 的节点
  for node in "${HEARTBEAT_NODES[@]}"; do
    if [ -z "$NAMESPACE" ]; then
      topic="/$node/heartbeat"
    else
      topic="$NAMESPACE/$node/heartbeat"
    fi

    echo "[$(date)] 检查心跳: $topic"

    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
      data_value=$(timeout 5 ros2 topic echo "$topic" --once 2>/dev/null | grep -o "data: [0-9]*" | awk '{print $2}')

      if [ -n "$data_value" ]; then
        echo "[$(date)]   $node 正常，心跳计数: $data_value"
      else
        echo "[$(date)]   $node 心跳数据丢失，重启中..."
        restart
        need_restart=1
        break
      fi
    else
      echo "[$(date)]   $node 心跳话题 $topic 不存在，重启中..."
      restart
      need_restart=1
      break
    fi
  done

  sleep $TIMEOUT
done
