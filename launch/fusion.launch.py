import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------- Arguments ----------------
    fusion_rate_hz = LaunchConfiguration('fusion_rate_hz')
    filter_type = LaunchConfiguration('filter_type')
    state_dim = LaunchConfiguration('state_dim')
    # Complementary
    comp_alpha = LaunchConfiguration('comp_alpha')
    # EKF
    ekf_process_noise = LaunchConfiguration('ekf_process_noise')

    declare_rate = DeclareLaunchArgument(
        'fusion_rate_hz',
        default_value='30.0',
        description='Rate at which to publish fused state.'
    )

    declare_filter = DeclareLaunchArgument(
        'filter_type',
        default_value='complementary',
        description='Filter strategy: complementary, ekf, ukf'
    )
    
    declare_dim = DeclareLaunchArgument(
        'state_dim',
        default_value='6',
        description='State vector dimension'
    )

    declare_alpha = DeclareLaunchArgument(
        'comp_alpha',
        default_value='0.98',
        description='Complementary filter alpha (weight for prediction/old_state)'
    )
    
    # Passing array as string often requires parsing in Node or passing as list. 
    # For simplicity in this lite lib, we handle it as a default list in the node if empty, 
    # or user can pass yaml. 
    # Here we won't strictly declare the array arg to avoid command line parsing issues 
    # unless using a yaml, but we'll show how to pass parameters.

    # ---------------- Node ----------------
    fusion_node = Node(
        package='sensor_fusion_lite',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[{
            'fusion_rate_hz': fusion_rate_hz,
            'filter_type': filter_type,
            'state_dim': state_dim,
            'complementary.alpha': comp_alpha,
            # 'ekf.initial_process_noise': [0.1, 0.1, ...], # Example of list passage
        }]
    )

    return LaunchDescription([
        declare_rate,
        declare_filter,
        declare_dim,
        declare_alpha,
        fusion_node
    ])
