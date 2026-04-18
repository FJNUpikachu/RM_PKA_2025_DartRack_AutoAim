import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('dart_bringup'), 'launch'))


def generate_launch_description():
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    from launch.actions import TimerAction
    from launch import LaunchDescription

    dart_bringup_share = get_package_share_directory('dart_bringup')

    launch_params = yaml.safe_load(open(os.path.join(
        dart_bringup_share,
        'config',
        'launch_params.yaml'
    )))

    def get_params(name):
        return os.path.join(
            dart_bringup_share,
            'config',
            'node_params',
            f'{name}_params.yaml'
        )

    # ===== 相机节点 =====
    hik_camera_node = ComposableNode(
        package='rm_hik_camera_driver',
        plugin='pka::hik_camera::HikCameraNode',
        name='hik_camera_driver',
        parameters=[get_params('hik_camera_driver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    dahua_camera_node = ComposableNode(
        package='rm_camera_driver',
        plugin='pka::camera_driver::Dahua_CameraNode',
        name='camera_driver',
        parameters=[get_params('dahua_camera_driver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ===== ONNX detector 节点 =====
    dart_detector_node = ComposableNode(
        package='dart_detector',
        plugin='pka::DartDetectorNode',
        name='dart_detector',
        parameters=[get_params('detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ===== 传统 detector 节点 =====
    traditional_detector_node = ComposableNode(
        package='dart_detector',
        plugin='pka::TraditionalDartDetectorNode',
        name='traditional_detector',
        parameters=[get_params('traditional_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ===== serial 节点 =====
    dart_serial_node = ComposableNode(
        package='dart_serial',
        plugin='pka::UARTNode',
        name='dart_serial',
        parameters=[get_params('serial')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ===== solver 节点 =====
    dart_solver_node = ComposableNode(
        package='dart_solver',
        plugin='pka::SolverNode',
        name='dart_solver_node',
        parameters=[get_params('solver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 根据 launch_params 选择相机
    def create_camera_container():
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

    # 根据 launch_params 选择 detector 版本
    # detector_version:
    #   1 -> ONNX 版
    #   2 -> 传统识别版
    def create_detector_container():
        detector_version = launch_params.get('detector_version', 1)
        detector_node = dart_detector_node if detector_version == 1 else traditional_detector_node

        return ComposableNodeContainer(
            name='detector_container',
            namespace=launch_params['namespace'],
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[detector_node],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args'],
        )

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

    delay_serial_container = TimerAction(
        period=1.5,
        actions=[serial_container],
    )

    delay_camera_container = TimerAction(
        period=2.0,
        actions=[create_camera_container()],
    )

    delay_detector_container = TimerAction(
        period=3.0,
        actions=[create_detector_container()],
    )

    delay_solver_container = TimerAction(
        period=3.5,
        actions=[solver_container],
    )

    return LaunchDescription([
        delay_serial_container,
        delay_camera_container,
        delay_detector_container,
        delay_solver_container
    ])