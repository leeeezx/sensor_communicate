from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor', # 替换成你的包名
            executable='force_sensor', # 假设你的主程序叫这个
            name='force_sensor_x_publisher',
            output='screen',
            parameters=[{'axis': 'x', 'port': '/dev/ttyUSB0'}]
        ),
        Node(
            package='sensor',
            executable='force_sensor',
            name='force_sensor_y_publisher',
            output='screen',
            parameters=[{'axis': 'y', 'port': '/dev/ttyUSB1'}]
        ),
        Node(
            package='sensor',
            executable='force_sensor',
            name='force_sensor_z_publisher',
            output='screen',
            parameters=[{'axis': 'z', 'port': '/dev/ttyUSB2'}]
        ),
        Node(
            package='rokae_move', # 你的C++节点所在的包
            executable='test_impedance_force_node', # 你的C++节点可执行文件名
            name='rokae_force_controller',
            output='screen'
        )
    ])