from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------
    # Launch arguments
    # -------------------------
    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Enable RViz"
    )

    # -------------------------
    # Paths to launch files
    # -------------------------
    description_launch = os.path.join(
        get_package_share_directory("crx30ia_description"),
        "launch",
        "bringup.launch.py"          
    )

    moveit_launch = os.path.join(
        get_package_share_directory("crx30ia_moveit_config"),
        "launch",
        "move_group.launch.py"      
    )

    rviz_launch = os.path.join(
        get_package_share_directory("crx30ia_moveit_config"),
        "launch",
        "moveit_rviz.launch.py"            
    )

    # -------------------------
    # Include launch files
    # -------------------------
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch)
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch)
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        condition=IfCondition(use_rviz)
    )

    # -------------------------
    # Final LaunchDescription
    # -------------------------
    return LaunchDescription([
        declare_use_rviz,
        description,
        moveit,
        rviz,
    ])
