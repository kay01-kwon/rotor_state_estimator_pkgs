import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rotor_state_estimator')

    estimator_config = os.path.join(pkg_share, 'config',
                                    'rotor_param.yaml')
    plotter_config = os.path.join(pkg_share, 'config',
                                  'plot_hexa.yaml')

    # Hexacopter CKF + RTS estimator
    estimator_node = Node(
        package='rotor_state_estimator',
        executable='ckf_node',
        name='ckf_node',
        output='screen',
        parameters=[estimator_config,
                    {'use_sim_time': False}],
    )

    # Real-time plotter (hexa mode)
    plotter_node = Node(
        package='rotor_state_estimator',
        executable='plot_rotor_state.py',
        name='rotor_state_plotter',
        output='screen',
        parameters=[plotter_config,
                    {'use_sim_time': False}],
    )

    return LaunchDescription([
        estimator_node,
        plotter_node,
    ])
