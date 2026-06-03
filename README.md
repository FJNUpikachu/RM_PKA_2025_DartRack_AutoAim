# Pikachu战队2026 DartTrack Vision Project

 本项目为福建师范大学PKA战队2026赛季飞镖镖架制导视觉部分，
 本项目基于2025赛季飞镖制导视觉部分编写。
 
 本项目中的神经网络和卡尔曼未调试成功，因此不可使用，只能使用传统视觉识别+一欧元滤波


## 一、项目结构

```
├── dart_detector : 识别部分
|
├── dart_solver : 解算部分
|
├── dart_bringup : 启动及参数文件
|
├── dart_interfaces : 自定义msg、srv
│
├── dart_serial : 与电控通信串口部分
│
├── dart_camera : 相机包
│
├── dart_upstart : 自启动
│
├── dart_utils : 工具包
```

## 二、环境配置

### Ubuntu 22.04

### 相机驱动

#### 海康相机SDK
海康相机免驱，可不用安装
保证该包内包含海康SDK即可。

#### 大华相机驱动（待补充）
!大华相机必须安装相机驱动!
*注：大华相机驱动安装过程麻烦，若明确不使用大华相机则可选择性安装。

内核版本：5.11.0及以下
编译器版本：gcc 11及以下

##### 降内核安装后升回
由于Ubuntu 22.04内核版本为6.5.0，需要导入Ubuntu20.04仓库源下载5.11.0版内核

  ```bash
  chmod +x MVviewer_Ver2.3.1_Linux_x86_Build20210926.run 
  sudo ./MVviewer_Ver2.3.1_Linux_x86_Build20210926.run
  ```

##### 编译器版本
建议先运行ros2一键安装，其gcc版本即11。

### ROS2 humble + OpenCV

*注：鱼香肉丝一键安装ros内有包含opencv 4.5.0，故无需自行安装opencv。
  ```bash
  wget http://fishros.com/install -O fishros && . fishros
  ```
安装时注意换源。
如此前安装过大华相机驱动，此时也可把旧源一起清理掉。

##### Onnxruntime安装

 ```bash
cd ~
wget https://github.com/microsoft/onnxruntime/releases/download/v1.24.4/onnxruntime-linux-x64-1.24.4.tgz
tar -xzf onnxruntime-linux-x64-1.24.4.tgz
sudo mv onnxruntime-linux-x64-1.24.4 /opt/onnxruntime
 ```

更新一下库
 ```bash
sudo ldconfig
 ```

## 三、编译与运行

```bash
rm -rf build install log
colcon build --symlink-install --parallel-workers 2

source install/setup.bash
ros2 launch dart_bringup bringup.launch.py
```

## 四、调试

- foxglove

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## 五、开机自启动
详情见dart_upstart README

## 六、上传github
```bash
git clone https://github.com/FJNUpikachu/RM_PKA_2025_DartRack_AutoAim.git
git add .
git commit -m "<提交说明>"
git push origin 2026
```

## 维护者及开源许可证


## 致谢
