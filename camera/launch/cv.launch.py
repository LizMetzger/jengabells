
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    cv_node = Node(
        package='camera',
        executable='cam',
        output='screen'
    )

    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch/rs_launch.py'
                ])
            ]),
        launch_arguments=[('depth_module.profile', '1280x720x30'),
                          ('pointcloud.enable', 'true'),
                          ('align_depth.enable', 'true')]
    )

    return LaunchDescription([
        launch_realsense,
        cv_node
    ])
