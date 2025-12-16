from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # 获取包路径
    go2_core_dir = get_package_share_directory('go2_core')
    go2_navigation_dir = get_package_share_directory('go2_navigation')
    go2_slam_dir = get_package_share_directory('go2_slam')
    go2_perception_dir = get_package_share_directory('go2_perception')

    try:
        pkg_share = get_package_share_directory('go2_description')
        xacro_file = os.path.join(pkg_share, 'urdf', 'go2_description.urdf.xacro')
    except Exception as e:
        print(f"Could not find go2_description: {e}")
        return ld

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=['0.30', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'utlidar']
    )

    # 1. 启动基础节点
    go2_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go2_core_dir, 'launch', 'go2_base.launch.py')
        ]),
        launch_arguments={
            'video_enable': 'false',
            'image_topic': '/camera/image_raw',
            'tcp_enable': 'true',
            'tcp_host': '127.0.0.1',
            'tcp_port': '5432',
            'target_fps': '30',
        }.items()
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 30
        }]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': False
        }]
    )

    # 2. 启动点云处理节点
    pointcloud_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go2_perception_dir, 'launch', 'go2_pointcloud_process.launch.py')
        ])
    )

    # 3. 启动SLAM工具箱
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go2_slam_dir, 'launch', 'go2_slamtoolbox.launch.py')
        ])
    )

    # 4. 启动导航系统
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go2_navigation_dir, 'launch', 'go2_nav2.launch.py')
        ])
    )

    map_manager = Node(
        package='map_manager',
        executable='map_manager_node',
        name='map_manager',
        parameters=[{
            'map_file': '/home/unitree/maps/auto_saved_map',
            'save_interval_seconds': 180.0  # Save every 3 minutes
        }],
        output='screen'
    )

    ld.add_action(go2_base_launch)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(lidar_tf)
    ld.add_action(pointcloud_process_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_launch)
    ld.add_action(map_manager)

    return ld
