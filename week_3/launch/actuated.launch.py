from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('week_3')
    models_path = os.path.join(pkg_path, 'models')

    sdf_file = os.path.join(pkg_path, 'models', 'arm', 'model.urdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[ 'gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    ) 

    # Load the robot into Gazebo using spawn_entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', sdf_file],
        output='screen'
    )

    # Setup state publisher node to publish TFs
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path),
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
        robot_state_publisher,
        position_controller
    ])
