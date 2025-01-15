import os
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('rm_description'), 'launch'))
def generate_launch_description():
    # 获取参数
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_description'), 'config', 'launch_params.yaml')))

    # 设置 robot_gimbal_description 命令
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=' + launch_params['odom2camera']['xyz'],
        ' rpy:=' + launch_params['odom2camera']['rpy']])

    # 发布机器人状态
    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                     'publish_frequency': 1000.0}]
    )

    return LaunchDescription([
        robot_gimbal_publisher,
        LogInfo(
            condition=None,
            msg="Starting robot URDF description and state publisher"
        )
    ])
