import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('dart_bringup'), 'launch'))


def generate_launch_description():
    # 修正导入 - 确保从正确的模块导入
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch_ros.descriptions import ComposableNode
    from launch.actions import TimerAction, Shutdown, GroupAction
    from launch import LaunchDescription

    # 加载启动参数
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('dart_bringup'), 'config', 'launch_params.yaml')))

    # 获取节点参数文件路径
    def get_params(name):
        return os.path.join(get_package_share_directory('dart_bringup'), 'config', 'node_params', f'{name}_params.yaml')

    # ========== 定义所有 ComposableNode ==========
    # 海康相机节点
    hik_camera_node = ComposableNode(
        package='rm_hik_camera_driver',
        plugin='pka::hik_camera::HikCameraNode',
        name='hik_camera_driver',
        parameters=[get_params('hik_camera_driver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # 大华相机节点
    dahua_camera_node = ComposableNode(
        package='rm_camera_driver',
        plugin='pka::camera_driver::Dahua_CameraNode',
        name='camera_driver',
        parameters=[get_params('dahua_camera_driver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 装甲板识别节点
    dart_detector_node = ComposableNode(
        package='dart_detector', 
        plugin='pka::DartDetectorNode',
        name='dart_detector',
        parameters=[get_params('detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 串口节点
    dart_serial_node = ComposableNode(
        package='dart_serial',
        plugin='pka::UARTNode',
        name='dart_serial',
        parameters=[get_params('serial')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
        
    # 装甲板解算节点
    dart_solver_node = ComposableNode(
        package='dart_solver',
        plugin='pka::SolverNode',
        name='dart_solver',
        parameters=[get_params('solver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ========== 创建各个独立的容器 ==========
    # 1. 相机容器（根据配置选择相机）
    def create_camera_container():
        # 根据配置选择相机节点
        camera_node = hik_camera_node if launch_params['camera'] == "hik" else dahua_camera_node
        
        return ComposableNodeContainer(
            name='camera_container',
            namespace=launch_params['namespace'],
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[camera_node],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args'],
        )

    # 2. 识别容器
    detector_container = ComposableNodeContainer(
        name='detector_container',
        namespace=launch_params['namespace'],
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[dart_detector_node],
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args'],
    )

    # 3. 串口节点容器
    serial_container = ComposableNodeContainer(
        name='serial_container',
        namespace=launch_params['namespace'],
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[dart_serial_node],
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args'],
    )

    # 4. 解算节点容器
    solver_container = ComposableNodeContainer(
        name='solver_container',
        namespace=launch_params['namespace'],
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[dart_solver_node],
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args'],
    )

    # ========== 延迟启动各个容器 ==========
    # 延迟启动串口容器（1.5秒）
    delay_serial_container = TimerAction(
        period=1.5,
        actions=[serial_container],
    )

    # 延迟启动相机容器（2.0秒）
    delay_camera_container = TimerAction(
        period=2.0,
        actions=[create_camera_container()],
    )

    # 延迟启动识别容器（3.0秒，在相机启动1秒后启动）
    delay_detector_container = TimerAction(
        period=3.0,
        actions=[detector_container],
    )

    # 延迟启动解算容器（3.5秒，在识别启动0.5秒后启动）
    delay_solver_container = TimerAction(
        period=3.5,
        actions=[solver_container],
    )

    # ========== 组装 LaunchDescription ==========
    launch_description_list = [
        # 延迟启动的容器
        delay_serial_container,
        delay_camera_container,
        delay_detector_container,
        delay_solver_container
    ]
    
    return LaunchDescription(launch_description_list)