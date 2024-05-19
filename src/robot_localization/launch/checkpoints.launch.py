from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    checkpoint = Node(
        package='robot_localization',
        executable='checkpoints',
        name='checkpoints',
        parameters=[{
            'checkpoints_file': get_package_share_directory('robot_localization')+'/launch/checkpoints.json'
        }],
    )

    ld = LaunchDescription()
    ld.add_action(checkpoint)
    return ld
