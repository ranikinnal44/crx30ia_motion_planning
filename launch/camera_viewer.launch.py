from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare launch arguments
    show_ee_camera_arg = DeclareLaunchArgument(
        'show_ee_camera',
        default_value='true',
        description='Show end-effector RGB camera'
    )

    show_ee_depth_arg = DeclareLaunchArgument(
        'show_ee_depth',
        default_value='true',
        description='Show end-effector depth camera'
    )

    show_scene_camera_arg = DeclareLaunchArgument(
        'show_scene_camera',
        default_value='true',
        description='Show scene RGB camera'
    )

    show_scene_depth_arg = DeclareLaunchArgument(
        'show_scene_depth',
        default_value='true',
        description='Show scene depth camera'
    )

    camera_viewer_node = Node(
        package='crx30ia_motion_planning',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'ee_color_topic': '/camera/color/image_raw',
            'ee_depth_topic': '/camera/depth/image_raw',
            'scene_color_topic': '/scene_camera/color/image_raw',
            'scene_depth_topic': '/scene_camera/depth/image_raw',
            'show_ee_camera': LaunchConfiguration('show_ee_camera'),
            'show_ee_depth': LaunchConfiguration('show_ee_depth'),
            'show_scene_camera': LaunchConfiguration('show_scene_camera'),
            'show_scene_depth': LaunchConfiguration('show_scene_depth'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        show_ee_camera_arg,
        show_ee_depth_arg,
        show_scene_camera_arg,
        show_scene_depth_arg,
        camera_viewer_node,
    ])

