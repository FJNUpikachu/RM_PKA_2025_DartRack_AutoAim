#!/bin/bash

# 获取当前用户的用户名和家目录
USER_NAME=${SUDO_USER:-$(whoami)}
HOME_DIR=$(eval echo ~$USER_NAME)
SERVICE_NAME="dart.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

# 检查是否为 root 用户，因为修改 systemd 配置需要 root 权限
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root."
    exit 1
fi

# 复制脚本到目标路径
chmod +x ./dart_clean_up.sh
chmod +x ./dart_watch_dog.sh
cp ./dart_clean_up.sh /usr/sbin/
cp ./dart_watch_dog.sh /usr/sbin/

# 动态创建 systemd service 文件内容
read -r -d '' SERVICE_CONTENT << EOF
[Unit]
Description=PKA2026 DartTrack Project
After=network.target

[Service]
User=$USER_NAME
Type=simple
ExecStart=/usr/sbin/dart_watch_dog.sh
ExecStop=/usr/sbin/dart_clean_up.sh
Restart=always
TimeoutStopSec=10
RestartSec=10s

[Install]
WantedBy=multi-user.target
EOF

# 创建/覆写 systemd service 文件
echo "Creating systemd service file at $SERVICE_PATH..."
echo "$SERVICE_CONTENT" > "$SERVICE_PATH"

# 重新加载 systemd 配置
echo "Reloading systemd daemon..."
systemctl daemon-reload

# 启用服务
echo "Enabling service $SERVICE_NAME..."
systemctl enable "$SERVICE_NAME"

# 启动服务
echo "Starting service $SERVICE_NAME..."
systemctl start "$SERVICE_NAME"

echo "Service $SERVICE_NAME has been registered and started."
