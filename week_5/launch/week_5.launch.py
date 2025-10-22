# Based on: https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace

import xml.etree.ElementTree as ET
import yaml

package_name = 'week_5'
launch_file_dir = PathJoinSubstitution([FindPackageShare(package_name), 'launch'])
pkg_gazebo_ros = FindPackageShare('gazebo_ros')

def gazebo_world(context : LaunchContext):

    limit_real_time_factor = eval(context.launch_configurations['limit_real_time_factor'].lower().capitalize())
    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'week_5.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world, 'verbose': 'false'}.items()
    )

    return [gzserver_cmd]

def group_action(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
    visualise_sensors = context.launch_configurations['visualise_sensors'].lower()
    
    # Can change SDF parameters here
    sdf_path = os.path.join(get_package_share_directory(package_name), 'models', 'waffle_pi', 'model.sdf')
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    robot_sdf = os.path.join(get_package_share_directory(package_name), 'models', 'robot.sdf')

    with open(robot_sdf, 'w') as f:
        tree.write(f, encoding='unicode')

    yaml_path = os.path.join(get_package_share_directory(package_name), 'config', 'initial_poses.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    with open(context.launch_configurations['rviz_windows'], 'r') as f:
        configuration = yaml.safe_load(f)

    rviz_windows = configuration[num_robots]

    bringup_cmd_group = []

    for robot_name, init_pose in initial_poses.items():
        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_file_dir, 'rviz.launch.py'])),
                condition=IfCondition(context.launch_configurations['use_rviz']),
                launch_arguments={'rviz_config': context.launch_configurations['rviz_config'],
                                  'window_x': str(rviz_windows[robot_name]['window_x']),
                                  'window_y': str(rviz_windows[robot_name]['window_y']),
                                  'window_width': str(rviz_windows[robot_name]['window_width']),
                                  'window_height': str(rviz_windows[robot_name]['window_height'])}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    launch_file_dir,
                    'spawn_robot.launch.py'])),
                launch_arguments={'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                  'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                  'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                  'robot_sdf': robot_sdf}.items())
        ])
    
        bringup_cmd_group.append(group)

    return bringup_cmd_group

def generate_launch_description():

    num_robots = LaunchConfiguration('num_robots')
    visualise_sensors = LaunchConfiguration('visualise_sensors')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    rviz_windows = LaunchConfiguration('rviz_windows')
    barrel_manager = LaunchConfiguration('barrel_manager')
    headless = LaunchConfiguration('headless')
    limit_real_time_factor = LaunchConfiguration('limit_real_time_factor')
    wait_for_barrels = LaunchConfiguration('wait_for_barrels')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_visualise_sensors_cmd = DeclareLaunchArgument(
        'visualise_sensors',
        default_value='true',
        description='Whether to visualise sensors in Gazebo')
            
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run the Gazebo GUI')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'namespaced.rviz']),
        description='Full path to the RViz config file to use')
    
    declare_rviz_windows_cmd = DeclareLaunchArgument(
        'rviz_windows',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'rviz_windows.yaml']),
        description='Full path to the RViz windows YAML file to use')        
    
    declare_barrel_manager_cmd = DeclareLaunchArgument(
        'barrel_manager',
        default_value='true',
        description='Whether to start the barrel manager')    
    
    declare_limit_real_time_factor_cmd = DeclareLaunchArgument(
        'limit_real_time_factor',
        default_value='True',
        description='Whether to limit the Gazebo real-time factor to 1.0')
    
    declare_wait_for_barrels_cmd = DeclareLaunchArgument(
        'wait_for_barrels',
        default_value='False',
        description='Whether to wait for every barrel to spawn before spawning any robots')
    
    gzserver_cmd = OpaqueFunction(function=gazebo_world)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
        condition=UnlessCondition(headless)
    )

    model_dir = os.path.join(
        get_package_share_directory(package_name),
        'models'
    )

    env_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ':' + model_dir
        ]
    )

    start_barell_manager_cmd = Node(
        package=package_name,
        executable='barrel_manager',
        output='screen',
        condition=IfCondition(barrel_manager)
    )

    bringup_cmd_group = OpaqueFunction(function=group_action)
        
    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    # Declare the launch options
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_visualise_sensors_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_windows_cmd)
    ld.add_action(declare_barrel_manager_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_limit_real_time_factor_cmd)
    ld.add_action(declare_wait_for_barrels_cmd)

    # Add the commands to the launch description
    ld.add_action(env_gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_barell_manager_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
