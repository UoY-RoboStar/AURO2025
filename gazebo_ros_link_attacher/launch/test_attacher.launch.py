import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros_link_attacher = get_package_share_directory('gazebo_ros_link_attacher')
    pkg_ros_link_attacher_prefix = get_package_prefix('gazebo_ros_link_attacher')

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[ 'gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', os.path.join(pkg_ros_link_attacher, 'worlds', 'test_attacher.world')],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    ) 

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_ros_link_attacher, 'worlds', 'test_attacher.world'),
            'pause': 'false',
        }.items()
    )

    return LaunchDescription([SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', os.path.join(pkg_ros_link_attacher_prefix, 'lib')), start_gazebo_server_cmd,start_gazebo_client_cmd])
