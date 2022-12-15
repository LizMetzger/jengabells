from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_path = get_package_share_path('camera')
    default_rviz_config_path = camera_path / 'april.rviz'
    default_april_config_path = camera_path / 'april.yaml'
    default_tf_config_path = camera_path / 'tf.yaml'

    rviz_launch_arg = DeclareLaunchArgument(
        name='rviz_pub',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable rviz2')

    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    cali_launch_arg = DeclareLaunchArgument(
        name='calibrate',
        default_value='false',
        choices=['true', 'false'],
        description='Launch calibration node')

    cv_node = Node(
        package='camera',
        executable='cam',
        output='screen',
        condition=LaunchConfigurationEquals('calibrate', 'false')
    )

    tf_broad_node = Node(
        package='camera',
        executable='broadcast',
        parameters=[default_tf_config_path],
        output='screen',
        condition=LaunchConfigurationEquals('calibrate', 'false')
    )

    cali_node = Node(
        package='camera',
        executable='cali',
        output='screen',
        condition=LaunchConfigurationEquals('calibrate', 'true')
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=LaunchConfigurationEquals('rviz_pub', 'true')
    )

    april_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        remappings=[('/image_rect', '/camera/color/image_raw'),
                    ('/camera_info', '/camera/color/camera_info')],
        parameters=[default_april_config_path],
        condition=LaunchConfigurationEquals('calibrate', 'true')
    )

    return LaunchDescription([
        launch_realsense,
        cali_launch_arg,
        rviz_config_arg,
        rviz_launch_arg,
        tf_broad_node,
        rviz_node,
        april_node,
        cv_node,
        cali_node
    ])
