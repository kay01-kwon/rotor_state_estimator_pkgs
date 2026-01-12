import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of the package
    rotor_state_estimator_pkg_share = get_package_share_directory('rotor_state_estimator')

    # Define the path to the configuration file
    config_file_path = os.path.join(rotor_state_estimator_pkg_share, 'config', 'rotor_param.yaml')
    print(f"Using configuration file: {config_file_path}")

    # Create the Node action for the ckf_node
    ckf_node = Node(
        package='rotor_state_estimator',
        executable='ckf_node',
        name='ckf_node',
        output='screen',
        parameters=[config_file_path,
                    {'use_sim_time': False}]
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        ckf_node
    ])