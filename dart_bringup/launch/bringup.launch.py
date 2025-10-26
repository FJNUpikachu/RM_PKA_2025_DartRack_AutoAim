import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

# 获取包路径
pkg_bringup_share = get_package_share_directory('dart_bringup')
sys.path.append(os.path.join(pkg_bringup_share, 'launch'))

def generate_launch_description():
    # 加载配置文件
    launch_params_path = os.path.join(pkg_bringup_share, 'config', 'launch_params.yaml')
    with open(launch_params_path, 'r') as f:
        launch_params = yaml.safe_load(f)
    
    namespace = launch_params.get('namespace', '')
    video_play = launch_params.get('video_play', False)  # 读取视频标志位
    
    # 获取相机配置文件路径
    hik_camera_pkg = get_package_share_directory('hik_camera')
    hik_params_file = os.path.join(hik_camera_pkg, 'config', 'camera_params.yaml')
    
    # 相机内参文件路径
    camera_info_path = os.path.join(pkg_bringup_share, 'config', 'camera_info.yaml')
    camera_info_url = f'file://{camera_info_path}'
    
    # 参数文件路径生成函数
    def get_params(param_file_name):
        return os.path.join(
            get_package_share_directory('dart_bringup'), 
            'config', 'node_params', 
            f'{param_file_name}_params.yaml'
        )
    
    # 1. 串口节点 (可组合节点)
    serial_node = ComposableNode(
        package='light_serial',
        plugin='light_serial::UARTNode',
        name='light_serial',
        parameters=[get_params('serial')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # 2. 解算节点 (可组合节点)
    solver_node = ComposableNode(
        package='dart_solver',
        plugin='dart_solver::SolverNode',
        name='dart_solver',
        parameters=[get_params('solver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # 3. 识别节点 (C++可组合节点)
    detector_node = ComposableNode(
        package='light_detector',
        plugin='dart_detector::DetectorNode',
        name='light_detector',
        parameters=[get_params('detector')],
        extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内通信
    )
    
    # 4. 海康相机节点 (可组合节点)
    hik_camera_node = ComposableNode(
        package='hik_camera',
        plugin='dart_camera::HikCameraNode',
        name='light_camera',
        parameters=[
            hik_params_file,
            {
                'camera_info_url': camera_info_url,
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内通信
    )

    # 创建相机与检测器专用容器，优化图像传输
    def get_camera_detector_container(*detector_nodes):
        nodes_list = list(detector_nodes)
        # 仅在不使用视频播放时添加相机节点
        if not video_play:
            nodes_list.append(hik_camera_node)
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_mt',  # 多线程容器
            composable_node_descriptions=nodes_list,
            output='both',
            emulate_tty=True,
            # 添加ROS参数以优化图像传输
            ros_arguments=['--ros-args', '-p', 'use_sim_time:=false'],
        )

    # 动态生成启动项
    def generate_launch_items(context, *args, **kwargs):
        # 主容器：包含串口和解算节点
        main_container = ComposableNodeContainer(
            name='main_component_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                serial_node,
                solver_node
            ],
            output='both',
            emulate_tty=True,
        )
        
        # 相机-检测器专用容器
        camera_detector_container = get_camera_detector_container(detector_node)
        
        launch_items = [
            PushRosNamespace(namespace),
            # 声明启动参数
            DeclareLaunchArgument(name='hik_params_file', default_value=hik_params_file),
            DeclareLaunchArgument(name='camera_info_url', default_value=camera_info_url),
            DeclareLaunchArgument(name='use_sensor_data_qos', default_value='false'),
            
            # 启动顺序：先启动主容器（串口节点）
            TimerAction(
                period=0.5,
                actions=[main_container]
            ),
            # 再启动相机-检测器容器（依赖串口初始化）
            TimerAction(
                period=1.5,
                actions=[camera_detector_container]
            )
        ]
        
        return launch_items
    
    return LaunchDescription([
        OpaqueFunction(function=generate_launch_items)
    ])
