from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = str(Path(get_package_share_directory('mpu9250_brazo')) / 'config' / 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abre la interfaz gráfica integrada.'),
        Node(package='mpu9250_brazo', executable='stm32_serial_node',
             name='stm32_serial_node', parameters=[params], output='screen'),
        Node(package='mpu9250_brazo', executable='calibration_node',
             name='imu1_calibration_node', parameters=[params], output='screen'),
        Node(package='mpu9250_brazo', executable='calibration_node',
             name='imu2_calibration_node', parameters=[params], output='screen'),
        Node(package='mpu9250_brazo', executable='orientation_node',
             name='imu1_orientation_node', parameters=[params], output='screen'),
        Node(package='mpu9250_brazo', executable='orientation_node',
             name='imu2_orientation_node', parameters=[params], output='screen'),
        Node(package='mpu9250_brazo', executable='robot_teleop_node',
             name='robot_teleop_node', parameters=[params], output='screen'),
        Node(package='mpu9250_brazo', executable='imu_gui_node',
             name='imu_gui_node', output='screen',
             condition=IfCondition(LaunchConfiguration('gui'))),
    ])
