# 记得改dart_watch_dog.sh里面的HOME_DIR

# 进入自启动目录：
cd ~/RM_PKA_2026_DartRack_AutoAim/src/dart_upstart

# 赋予执行权限：
chmod +x register_service.sh
chmod +x dart_watch_dog.sh
chmod +x dart_clean_up.sh

# 在 dart_upstart 目录下执行：
cd ~/RM_PKA_2026_DartRack_AutoAim/src/dart_upstart
sudo ./register_service.sh

# 验证自启动是否成功:
systemctl status dart.service
如果看到：Active: active (running)
说明自启动服务正在运行。

# 如果只是想临时停止当前正在运行的服务：
sudo systemctl stop dart.service
注意：
只执行 stop 不会取消开机自启动。下次重启后它还会自动启动。

# 如果不希望系统开机后自动启动飞镖程序，执行：
sudo systemctl disable dart.service

# 如果之后又想恢复开机自启动：
sudo systemctl enable dart.service
sudo systemctl start dart.service

# 如果你不仅想取消开机自启动，还想彻底删除 systemd 服务文件：
sudo systemctl stop dart.service
sudo systemctl disable dart.service
sudo rm -f /etc/systemd/system/dart.service
sudo systemctl daemon-reload

# 如果注册脚本把 watchdog 和 clean 脚本复制到了 /usr/sbin/，也可以一起删除：
sudo rm -f /usr/sbin/dart_watch_dog.sh
sudo rm -f /usr/sbin/dart_clean_up.sh