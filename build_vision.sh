colcon build --symlink-install --parallel-workers 2

source install/setup.bash
ros2 launch dart_bringup bringup.launch.py