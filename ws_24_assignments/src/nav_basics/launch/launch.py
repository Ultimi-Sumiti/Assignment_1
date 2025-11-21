import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import GroupAction

def generate_launch_description():

    pkg_name = 'nav_basics'

    assignment1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ir_launch'),
                'launch',
                'assignment_1.launch.py'
            ])
        ])
    )

    apriltag_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'launch_AprilTag_node.yml'
            ])
        )
    )

    lifecycle_nodes_client = Node(
        package=pkg_name,
        executable='lifecycle_nodes_client',
        name='lifecycle_nodes_client',
        output='screen',
        emulate_tty=True
    )

    # The two nodes below must start after lifecycle_nodes_client.
    dependent_nodes = GroupAction(
        actions=[
            Node(
                package=pkg_name,
                executable='nav2_action_client',
                name='nav2_action_client',
                output='screen',
                emulate_tty=True
            ),
            
            Node(
                package=pkg_name,
                executable='perception_node',
                name='perception_node',
                output='screen',
                emulate_tty=True
            ),
        ]
    )

    delay_dependent_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_nodes_client,
            on_exit=[dependent_nodes],
        )
    )

    # Create launch descriptor.
    ld = LaunchDescription()
    
    ld.add_action(assignment1_launch)
    ld.add_action(apriltag_launch)
    ld.add_action(lifecycle_nodes_client)
    ld.add_action(delay_dependent_launch)

    return ld
