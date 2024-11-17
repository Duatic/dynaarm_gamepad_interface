from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Argument for the config file
        DeclareLaunchArgument(
            'config_file',
            default_value='config/gamepad_config.yaml',
            description='Path to the gamepad configuration YAML file'
        ),

        # Node for the gamepad receiver
        Node(
            package='gamepad_interface',
            executable='gamepad_node',
            name='gamepad_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
    ])