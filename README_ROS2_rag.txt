警告⚠️：ROS2_bag不能运行过长时间，会爆硬盘!

1.手动启动ROS2 bag方法：

启动主程序，确认话题已经出来后，再开一个终端执行：
bash start_dart_bag.sh

停止录制：
Ctrl + C

查看录制结果：
ros2 bag info /home/pka/rm_bags/你的bag文件夹名

回放：
ros2 bag play /home/pka/rm_bags/你的bag文件夹名


2.开机自启动：

进入工程目录：
cd /home/pka/RM_PKA_2026_DartRack_AutoAim

给执行权限：
chmod +x start_dart_bag.sh

创建 systemd 服务文件：
sudo nano /etc/systemd/system/dart_bag_record.service

写入下面内容：
[Unit]
Description=ROS2 Bag Recorder for Dart Traditional Vision
After=network-online.target dart.service
Wants=network-online.target dart.service

[Service]
Type=simple
User=pka
WorkingDirectory=/home/pka/RM_PKA_2026_DartRack_AutoAim
ExecStart=/bin/bash /home/pka/RM_PKA_2026_DartRack_AutoAim/start_dart_bag.sh
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=30
Environment=RCUTILS_LOGGING_BUFFERED_STREAM=1
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target

重新加载 systemd
sudo systemctl daemon-reload

设置开机自启：
sudo systemctl enable dart_bag_record.service

查看是否启用成功：
systemctl is-enabled dart_bag_record.service
如果输出：enabled 就说明开机自启设置成功。

取消开机自启：
sudo systemctl disable dart_bag_record.service

查看是否开机自启：
systemctl is-enabled dart_bag_record.service
