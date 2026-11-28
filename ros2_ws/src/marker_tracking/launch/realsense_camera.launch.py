import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Uncommented to load the config
    pkg_dir = get_package_share_directory('marker_tracking')
    realsense_config = os.path.join(pkg_dir, 'config', 'realsense_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Launch RealSense camera driver',
        ),
        DeclareLaunchArgument(
            'camera_serial_no',
            default_value='',
            description='RealSense camera serial number',
        ),

        # RealSense camera driver (from upstream package)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py',
            ]),
            launch_arguments={
                # 'align_depth.enable': 'true',
                # 'pointcloud.enable': 'false',
                
                # Load the realsense camera config
                'config_file': realsense_config,
                'serial_no': LaunchConfiguration('camera_serial_no'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),

        # TODO: detection node
    ])