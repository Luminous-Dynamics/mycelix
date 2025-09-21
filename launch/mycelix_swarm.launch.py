"""
Mycelix ROS2 Swarm Launch File
Launches multiple robot agents connected to the consciousness network
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for Mycelix swarm"""
    
    # Launch arguments
    robot_count = LaunchConfiguration('robot_count', default='3')
    namespace_base = LaunchConfiguration('namespace_base', default='robot')
    holochain_url = LaunchConfiguration('holochain_url', default='ws://localhost:8888')
    enable_visualization = LaunchConfiguration('enable_visualization', default='true')
    
    # Package paths
    mycelix_bridge_dir = get_package_share_directory('mycelix_ros2_bridge')
    config_file = os.path.join(mycelix_bridge_dir, 'config', 'swarm_config.yaml')
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            'robot_count',
            default_value='3',
            description='Number of robots in the swarm'
        )
    )
    
    ld.add_action(
        DeclareLaunchArgument(
            'namespace_base',
            default_value='robot',
            description='Base namespace for robots'
        )
    )
    
    ld.add_action(
        DeclareLaunchArgument(
            'holochain_url',
            default_value='ws://localhost:8888',
            description='Holochain conductor WebSocket URL'
        )
    )
    
    ld.add_action(
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable visualization tools'
        )
    )
    
    # Start Holochain conductor (if not already running)
    ld.add_action(
        ExecuteProcess(
            cmd=['hc', 'sandbox', 'run', '--ports', '8888'],
            output='screen',
            shell=True,
            prefix=['gnome-terminal', '--tab', '--title="Holochain Conductor"', '--', 'bash', '-c']
        )
    )
    
    # Launch robots dynamically based on count
    # For demonstration, we'll launch 3 robots with different configurations
    robot_configs = [
        {
            'name': 'robot_001',
            'model': 'turtlebot3_waffle',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'capabilities': 'navigation,mapping,object_detection'
        },
        {
            'name': 'robot_002',
            'model': 'turtlebot3_burger',
            'x': 2.0,
            'y': 0.0,
            'z': 0.0,
            'capabilities': 'navigation,exploration'
        },
        {
            'name': 'robot_003',
            'model': 'turtlebot3_waffle_pi',
            'x': -2.0,
            'y': 0.0,
            'z': 0.0,
            'capabilities': 'navigation,mapping,vision,manipulation'
        }
    ]
    
    # Launch each robot
    for i, robot_config in enumerate(robot_configs):
        robot_namespace = f"{namespace_base}_{i+1:03d}"
        
        # Mycelix bridge node for each robot
        ld.add_action(
            Node(
                package='mycelix_ros2_bridge',
                executable='mycelix_bridge_node',
                name='mycelix_bridge',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    {
                        'robot_id': robot_config['name'],
                        'robot_model': robot_config['model'],
                        'holochain_url': holochain_url,
                        'learning_rate': 0.01,
                        'privacy_epsilon': 1.0,
                        'experience_buffer_size': 1000,
                        'capabilities': robot_config['capabilities']
                    }
                ],
                remappings=[
                    ('/scan', f'/{robot_namespace}/scan'),
                    ('/odom', f'/{robot_namespace}/odom'),
                    ('/imu', f'/{robot_namespace}/imu'),
                    ('/cmd_vel', f'/{robot_namespace}/cmd_vel'),
                    ('/camera/image_raw', f'/{robot_namespace}/camera/image_raw')
                ]
            )
        )
        
        # Robot state publisher for visualization
        ld.add_action(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_namespace,
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_config['model'],
                        'use_sim_time': True
                    }
                ]
            )
        )
        
        # Spawn robot in Gazebo (if simulation)
        ld.add_action(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{robot_config["name"]}',
                output='screen',
                arguments=[
                    '-entity', robot_config['name'],
                    '-topic', f'/{robot_namespace}/robot_description',
                    '-x', str(robot_config['x']),
                    '-y', str(robot_config['y']),
                    '-z', str(robot_config['z']),
                    '-robot_namespace', robot_namespace
                ]
            )
        )
    
    # Swarm coordinator node
    ld.add_action(
        Node(
            package='mycelix_ros2_bridge',
            executable='swarm_coordinator',
            name='swarm_coordinator',
            output='screen',
            parameters=[
                {
                    'swarm_size': 3,
                    'coordination_frequency': 5.0,  # Hz
                    'consensus_threshold': 0.66,    # 2/3 majority
                    'formation_type': 'triangle',
                    'holochain_url': holochain_url
                }
            ]
        )
    )
    
    # Visualization tools
    if enable_visualization:
        # RViz2 for visualization
        ld.add_action(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(mycelix_bridge_dir, 'rviz', 'swarm.rviz')]
            )
        )
        
        # Consciousness field visualizer
        ld.add_action(
            Node(
                package='mycelix_ros2_bridge',
                executable='consciousness_visualizer',
                name='consciousness_visualizer',
                output='screen',
                parameters=[
                    {
                        'update_rate': 10.0,
                        'field_resolution': 0.1,
                        'visualization_mode': 'resonance'
                    }
                ]
            )
        )
        
        # PlotJuggler for real-time plotting
        ld.add_action(
            ExecuteProcess(
                cmd=['plotjuggler'],
                output='screen'
            )
        )
    
    # Monitoring dashboard
    ld.add_action(
        ExecuteProcess(
            cmd=['python3', os.path.join(mycelix_bridge_dir, 'scripts', 'monitoring_dashboard.py')],
            output='screen',
            shell=True,
            prefix=['gnome-terminal', '--tab', '--title="Mycelix Dashboard"', '--', 'bash', '-c']
        )
    )
    
    # Log swarm initialization
    ld.add_action(
        LogInfo(
            msg='🍄 Mycelix Swarm Initialized! 🍄\n'
                '================================\n'
                'Robots are connecting to the consciousness network...\n'
                'Federated learning will begin in 60 seconds.\n'
                'Swarm coordination active at 5Hz.\n'
                '================================'
        )
    )
    
    return ld