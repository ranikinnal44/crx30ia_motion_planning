from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "crx30ia",
        package_name="crx30ia_moveit_config"
    ).to_moveit_configs()

    common_params = [
        moveit_config.to_dict(),
        {"use_sim_time": True},
    ]

    simple_test_node = Node(
        package="crx30ia_motion_planning",
        executable="simple_goal_test",
        output="screen",
        parameters=common_params,
    )

    return LaunchDescription([
        simple_test_node,
    ])

