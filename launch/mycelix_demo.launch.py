#!/usr/bin/env python3
"""
Mycelix ROS2 Bridge Demo Launch File
Launches multiple robots with consciousness network integration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    num_robots = LaunchConfiguration('num_robots', default='3')
    holochain_url = LaunchConfiguration('holochain_url', default='ws://localhost:8888')
    enable_visualization = LaunchConfiguration('enable_visualization', default='true')
    formation_type = LaunchConfiguration('formation_type', default='triangle')
    privacy_epsilon = LaunchConfiguration('privacy_epsilon', default='1.0')
    
    # Launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots in the swarm'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'holochain_url',
        default_value='ws://localhost:8888',
        description='Holochain conductor WebSocket URL'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable consciousness field visualization'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'formation_type',
        default_value='triangle',
        description='Swarm formation type (triangle, circle, line, square, v_formation)'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'privacy_epsilon',
        default_value='1.0',
        description='Differential privacy epsilon for federated learning'
    ))
    
    # Launch Holochain conductor (optional, can be started separately)
    ld.add_action(LogInfo(msg='🍄 Starting Mycelix Consciousness Network Demo'))
    
    # Launch robots with Mycelix bridge
    for i in range(1, 4):  # Launch 3 robots by default
        robot_id = f"robot_{i:03d}"
        
        # Mycelix Bridge Node
        ld.add_action(Node(
            package='mycelix_ros2_bridge',
            executable='mycelix_bridge_node',
            name=f'{robot_id}_bridge',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'robot_model': 'turtlebot3',
                'holochain_url': holochain_url,
                'learning_rate': 0.01,
                'privacy_epsilon': privacy_epsilon,
                'experience_buffer_size': 1000,
            }],
            remappings=[
                ('/cmd_vel', f'/{robot_id}/cmd_vel'),
                ('/odom', f'/{robot_id}/odom'),
                ('/scan', f'/{robot_id}/scan'),
                ('/imu', f'/{robot_id}/imu'),
            ]
        ))
        
        # Robot simulator (placeholder - replace with actual robot launch)
        ld.add_action(Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            name=f'{robot_id}_simulator',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'robot_name': robot_id,
                'initial_pose_x': float(i - 2) * 2.0,
                'initial_pose_y': 0.0,
                'initial_pose_z': 0.0,
            }]
        ))
    
    # Launch Swarm Coordinator
    ld.add_action(Node(
        package='mycelix_ros2_bridge',
        executable='swarm_coordinator',
        name='swarm_coordinator',
        output='screen',
        parameters=[{
            'swarm_size': 3,
            'coordination_frequency': 5.0,
            'consensus_threshold': 0.66,
            'formation_type': formation_type,
            'holochain_url': holochain_url,
            'formation_spacing': 2.0,
            'max_swarm_velocity': 1.0,
        }]
    ))
    
    # Launch Consciousness Visualizer
    ld.add_action(Node(
        package='mycelix_ros2_bridge',
        executable='consciousness_visualizer',
        name='consciousness_visualizer',
        condition=IfCondition(enable_visualization),
        output='screen',
        parameters=[{
            'update_rate': 10.0,
            'field_resolution': 0.5,
            'visualization_mode': 'resonance',
            'field_size': 20.0,
            'harmonic_base': 432.0,
            'show_particles': True,
            'show_field': True,
            'show_connections': True,
        }]
    ))
    
    # Launch RViz for visualization
    rviz_config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'mycelix_visualization.rviz'
    )
    
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(enable_visualization),
        arguments=['-d', rviz_config],
        output='screen'
    ))
    
    # Launch monitoring tools
    ld.add_action(Node(
        package='mycelix_ros2_bridge',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'safety_check_rate': 10.0,
            'max_velocity': 2.0,
            'min_obstacle_distance': 0.5,
            'emergency_stop_distance': 0.2,
        }]
    ))
    
    # Performance monitor
    ld.add_action(Node(
        package='mycelix_ros2_bridge',
        executable='performance_monitor',
        name='performance_monitor',
        output='screen',
        parameters=[{
            'monitor_rate': 1.0,
            'log_to_file': True,
            'log_directory': '/tmp/mycelix_logs',
        }]
    ))
    
    return ld