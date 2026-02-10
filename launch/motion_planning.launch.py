import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "crx30ia",
        package_name="crx30ia_moveit_config"
    ).to_moveit_configs()

    move_to_pose_node = Node(
        package="crx30ia_motion_planning",
        executable="move_to_pose",
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

    pick_place_node = Node(
        package="crx30ia_motion_planning",
        executable="pick_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    grasp_node = Node(
        package="crx30ia_motion_planning",
        executable="grasp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    setup_scene_node = Node(
        package="crx30ia_motion_planning",
        executable="setup_scene",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_to_pose_node,
        # grasp_node,
        # publish_goal_node,
        # setup_scene_node,
        # pick_place_node,
        # trace_waypoints_node,
        # publish_waypoints_node,
    ])
