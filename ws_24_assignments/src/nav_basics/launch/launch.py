import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import  TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():

    pkg_name = 'nav_basics'

    external_launch = IncludeLaunchDescription(
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
        output='screen'
    )

    nav2_action_client = Node(
        package=pkg_name,
        executable='nav2_action_client',
        name='nav2_action_client',
        output='screen'
    )

    perception_node = Node(
        package=pkg_name,
        executable='perception_node',
        name='perception_node',
        output='screen'
    )

    delayed_nav2_action_client= TimerAction(
        period=4.0,
        actions=[nav2_action_client] # This start the node with 4 seconds of retard, to let the server catch some apple.
    )


    ld = LaunchDescription()
    
    ld.add_action(external_launch)
    ld.add_action(apriltag_launch)
    ld.add_action(lifecycle_nodes_client)
    ld.add_action(delayed_nav2_action_client)
    ld.add_action(perception_node)

    return ld