#!/usr/bin/env python3
"""
Three Agent Demo Launch File for Mycelix-Holochain
Demonstrates collective consciousness and federated learning
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """Generate launch description for 3-agent swarm demonstration."""
    
    # Declare launch arguments
    swarm_size = LaunchConfiguration('swarm_size', default='3')
    holochain_url = LaunchConfiguration('holochain_url', default='ws://localhost:8888')
    visualization_mode = LaunchConfiguration('viz_mode', default='harmonic')
    
    # Robot namespace prefixes
    robot_namespaces = ['robot_001', 'robot_002', 'robot_003']
    
    # List to hold all nodes
    nodes = []
    
    # Create federated learner nodes for each robot
    for i, ns in enumerate(robot_namespaces):
        # Federated Learner Node
        learner_node = Node(
            package='mycelix_bridge',
            executable='federated_learner_main',
            name=f'federated_learner_{ns}',
            namespace=ns,
            parameters=[{
                'learning_rate': 0.001,
                'privacy_epsilon': 1.0,
                'batch_size': 32,
                'robot_count': 3,
                'model_type': 'simple',
                'aggregation_method': 'byzantine' if i == 0 else 'fedavg',
                'enable_uncertainty': True,
                'hierarchical_fl': False,
            }],
            remappings=[
                ('/mycelix/local_gradients', f'/{ns}/mycelix/local_gradients'),
                ('/mycelix/global_model', '/mycelix/global_model'),  # Shared topic
                ('/mycelix/model_metrics', f'/{ns}/mycelix/model_metrics'),
            ],
            output='screen',
            emulate_tty=True,
        )
        
        # Holochain Agent Node
        agent_node = Node(
            package='mycelix_bridge',
            executable='holochain_agent_node',
            name=f'holochain_agent_{ns}',
            namespace=ns,
            parameters=[{
                'agent_id': ns,
                'conductor_url': holochain_url,
                'model': f'TurtleBot3_{i+1}',
                'manufacturer': 'ROBOTIS',
                'ros_version': 'humble',
                'base_frequency': 432.0 + (i * 8.0),  # Unique frequency per robot
                'capabilities': ['navigation', 'mapping', 'learning', 'consciousness'],
            }],
            output='screen',
            emulate_tty=True,
        )
        
        # Simulated robot state publisher (for demo)
        robot_sim_node = Node(
            package='mycelix_bridge',
            executable='robot_simulator',
            name=f'robot_sim_{ns}',
            namespace=ns,
            parameters=[{
                'robot_id': ns,
                'initial_x': -2.0 + (i * 2.0),  # Spread robots horizontally
                'initial_y': 0.0,
                'movement_pattern': ['circular', 'linear', 'random'][i],
                'update_rate': 10.0,
            }],
            output='screen',
        )
        
        nodes.extend([
            TimerAction(
                period=float(i * 2.0),  # Stagger startup
                actions=[learner_node, agent_node, robot_sim_node]
            )
        ])
    
    # Swarm Coordinator (single instance for all robots)
    swarm_coordinator = Node(
        package='mycelix_bridge',
        executable='swarm_coordinator_main',
        name='swarm_coordinator',
        parameters=[{
            'swarm_size': 3,
            'coordination_frequency': 5.0,
            'consensus_threshold': 0.66,
            'formation_type': 'triangle',
            'formation_spacing': 2.0,
            'max_swarm_velocity': 1.0,
            'holochain_url': holochain_url,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # Consciousness Visualizer
    consciousness_viz = Node(
        package='mycelix_bridge',
        executable='consciousness_visualizer_main',
        name='consciousness_visualizer',
        parameters=[{
            'update_rate': 10.0,
            'field_resolution': 0.5,
            'visualization_mode': visualization_mode,
            'field_size': 20.0,
            'harmonic_base': 432.0,
            'show_particles': True,
            'show_field': True,
            'show_connections': True,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # Demo Controller - Orchestrates the demonstration
    demo_controller = Node(
        package='mycelix_bridge',
        executable='demo_controller',
        name='demo_controller',
        parameters=[{
            'demo_duration': 300,  # 5 minutes
            'scenario_sequence': [
                'exploration',      # 0-60s: Random exploration
                'formation',        # 60-120s: Triangle formation
                'collective_task',  # 120-180s: Collective transport
                'consensus',        # 180-240s: Consensus decision
                'emergence',        # 240-300s: Emergent behavior
            ],
        }],
        output='screen',
    )
    
    # RViz2 for visualization
    rviz_config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'mycelix_demo.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
    )
    
    # Performance Monitor
    performance_monitor = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/mycelix/swarm_metrics'],
        output='screen',
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('swarm_size', default_value='3',
                            description='Number of robots in swarm'),
        DeclareLaunchArgument('holochain_url', default_value='ws://localhost:8888',
                            description='Holochain conductor WebSocket URL'),
        DeclareLaunchArgument('viz_mode', default_value='harmonic',
                            description='Visualization mode for consciousness field'),
        
        # Log startup message
        LogInfo(msg=[
            '\n',
            '=' * 70, '\n',
            '🐝 MYCELIX THREE AGENT DEMONSTRATION\n',
            '=' * 70, '\n',
            '🧠 Federated Learning: Privacy-preserving collective intelligence\n',
            '🌐 Holochain Network: Distributed consensus and validation\n',
            '✨ Consciousness Field: Quantum entanglement visualization\n',
            '🤝 Swarm Coordination: Multi-robot formation control\n',
            '=' * 70, '\n',
            '\n',
            'Demo Sequence (5 minutes):\n',
            '  0:00-1:00 - Exploration Phase\n',
            '  1:00-2:00 - Formation Control\n',
            '  2:00-3:00 - Collective Task\n',
            '  3:00-4:00 - Consensus Decision\n',
            '  4:00-5:00 - Emergent Behavior\n',
            '\n',
            '=' * 70, '\n',
        ]),
        
        # Core infrastructure nodes
        TimerAction(
            period=0.0,
            actions=[swarm_coordinator, consciousness_viz]
        ),
        
        # Robot nodes (staggered startup)
        *nodes,
        
        # Demo controller and monitoring
        TimerAction(
            period=5.0,  # Wait for infrastructure
            actions=[demo_controller]
        ),
        
        # Visualization
        TimerAction(
            period=3.0,
            actions=[rviz_node]
        ),
        
        # Performance monitoring
        TimerAction(
            period=10.0,
            actions=[performance_monitor]
        ),
    ])