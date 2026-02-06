import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "crx30ia",
        package_name="crx30ia_moveit_config"
    ).to_moveit_configs()

    simple_goal_node = Node(
        package="crx30ia_motion_planning",
        executable="simple_goal",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )


    publish_goal_node = Node(
            package="crx30ia_motion_planning",
            executable="publish_goal",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True},
        ],
    )
    
    trace_waypoints_node = Node(
        package="crx30ia_motion_planning",
        executable="trace_waypoints",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    publish_waypoints_node = Node(
        package="crx30ia_motion_planning",
        executable="publish_waypoints",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        simple_goal_node,
        publish_goal_node,
        # trace_waypoints_node,
        # publish_waypoints_node,
    ])
