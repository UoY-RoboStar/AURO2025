# Based on: https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py
import os
import psutil
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext, Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, OpaqueFunction, SetEnvironmentVariable, EmitEvent
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace, SetUseSimTime
from typing import List

import xml.etree.ElementTree as ET
import yaml

package_name = 'assessment'
launch_file_dir = PathJoinSubstitution([FindPackageShare(package_name), 'launch'])
pkg_gazebo_ros = FindPackageShare('gazebo_ros')
default_rviz_config = os.path.join(get_package_share_directory('assessment'), 'rviz', 'namespaced.rviz')

class EmptyAction(Action):
    def execute(self, context: LaunchContext) -> List[Action]:
        # Do nothing
        return []

def get_gazebo_pid():
    """Return the PID of the first running gzserver process, or None if not running."""
    for proc in psutil.process_iter(['name', 'pid']):
        if proc.info['name'] == 'gzserver':
            return proc.info['pid']
    return None   

def gazebo_world(context : LaunchContext):

    obstacles = eval(context.launch_configurations['obstacles'].lower().capitalize())
    limit_real_time_factor = eval(context.launch_configurations['limit_real_time_factor'].lower().capitalize())

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'assessment.world')
    tree = ET.parse(world_path)
    root = tree.getroot()

    if obstacles == False:
        for world in root.findall('.//world'):
            for model in world.findall('.//model'):
                if "box" in model.attrib['name'] or "cylinder" in model.attrib['name']:
                    world.remove(model)

    if limit_real_time_factor == False:
        for node in root.iter("real_time_update_rate"):
            for element in node.iter():
                element.text = "0.0"

    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'simulation.world')

    with open(world, 'w') as f:
        tree.write(f, encoding='unicode')

    gzserver_pid = get_gazebo_pid()

    if gzserver_pid is None:
        gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
            ),
            launch_arguments={'world': world, 'force_system': 'False', 'verbose': context.launch_configurations['gazebo_verbose'].lower()}.items()
        )
        return [gzserver_cmd]
    else:
        return [
                LogInfo(msg=f"[ERROR] Gazebo is already running (PID: {gzserver_pid}). Consider killing it first using 'pkill -9 gzserver' and then re-launch."),
                EmitEvent(event=Shutdown())
            ]

def group_action(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
    visualise_sensors = context.launch_configurations['visualise_sensors'].lower()
    odometry_source = context.launch_configurations['odometry_source']
    sensor_noise = eval(context.launch_configurations['sensor_noise'].lower().capitalize())
    initial_pose_package = context.launch_configurations['initial_pose_package']
    initial_pose_file = context.launch_configurations['initial_pose_file']
    map_file = context.launch_configurations['map']

    # Can change SDF parameters here
    sdf_path = os.path.join(get_package_share_directory(package_name), 'models', 'waffle_pi', 'model.sdf')
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for node in root.iter("visualize"):
        for element in node.iter():
            element.text = visualise_sensors

    for node in root.iter("odometry_source"):
        for element in node.iter():
            if odometry_source == "ENCODER":
                element.text = "0"
            elif odometry_source == "WORLD":
                element.text = "1"

    if sensor_noise == False:

        for sensor in root.findall('.//sensor'):
            for imu in sensor.findall('.//imu'):
                sensor.remove(imu)

        for ray in root.findall('.//ray'):
            for noise in ray.findall('.//noise'):
                ray.remove(noise)

        for camera in root.findall('.//camera'):
            for noise in camera.findall('.//noise'):
                camera.remove(noise)

    robot_sdf = os.path.join(get_package_share_directory(package_name), 'models', 'robot.sdf')

    with open(robot_sdf, 'w') as f:
        tree.write(f, encoding='unicode')

    yaml_path = os.path.join(get_package_share_directory(initial_pose_package), initial_pose_file)

    print('pose: ' + yaml_path)

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    if num_robots > 0:
        initial_poses = configuration[num_robots]
    else:
        return [EmptyAction()]

    with open(context.launch_configurations['rviz_windows'], 'r') as f:
        configuration = yaml.safe_load(f)

    if num_robots > 0:
        rviz_windows = configuration[num_robots]
    else:
        rviz_windows = {}

    bringup_cmd_group = []

    rviz_config = context.launch_configurations['rviz_config']
    rviz_log_level = context.launch_configurations['rviz_log_level']

    # Load correct RViz config automatically if using use_nav2 parameter, and unless a differnt default has been set!
    if eval(context.launch_configurations['use_nav2'].lower().capitalize()):
        if rviz_config == default_rviz_config:
            rviz_config = os.path.join(get_package_share_directory('assessment'), 'rviz', 'namespaced_nav2.rviz')

        if not os.path.exists(map_file):
            return [
                LogInfo(msg=f"[ERROR] use_nav2 specified, but map file '{map_file}' does not exist. Did you forget to provide a map file?"),
                EmitEvent(event=Shutdown())
            ]

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
                launch_arguments={'rviz_config': rviz_config,
                                  'window_x': str(rviz_windows[robot_name]['window_x']),
                                  'window_y': str(rviz_windows[robot_name]['window_y']),
                                  'window_width': str(rviz_windows[robot_name]['window_width']),
                                  'window_height': str(rviz_windows[robot_name]['window_height']),
                                  'log_level': rviz_log_level}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    launch_file_dir,
                    'spawn_robot.launch.py'])),
                launch_arguments={'use_nav2': context.launch_configurations['use_nav2'],
                                  'map': context.launch_configurations['map'],
                                  'params_file': context.launch_configurations['params_file'],
                                  'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                  'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                  'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                  'robot_sdf': robot_sdf}.items())
            ,
            Node(
                package=package_name,
                executable='visual_sensor',
                output='screen',
                parameters=[{'publish_image_barrels': eval(context.launch_configurations['vision_sensor_debug']), 
                             'publish_image_zones': eval(context.launch_configurations['vision_sensor_debug']), 
                             'skip_similar_frames': eval(context.launch_configurations['vision_sensor_skip_frames']), 
                             'publish_image_zones_mask': eval(context.launch_configurations['vision_sensor_debug']),
                             'frame_divider': float(context.launch_configurations['vision_sensor_frame_divider'])}],
                condition=IfCondition(context.launch_configurations['vision_sensor'])             
                ),
            Node(
                package='assessment',
                executable='dynamic_mask.py',
                output='screen'
            )
        ])
    
        bringup_cmd_group.append(group)

    return bringup_cmd_group

def generate_launch_description():

    num_robots = LaunchConfiguration('num_robots')
    visualise_sensors = LaunchConfiguration('visualise_sensors')
    odometry_source = LaunchConfiguration('odometry_source')
    sensor_noise = LaunchConfiguration('sensor_noise')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    rviz_config_file = LaunchConfiguration('rviz_config')
    rviz_windows = LaunchConfiguration('rviz_windows')
    obstacles = LaunchConfiguration('obstacles')
    barrel_manager = LaunchConfiguration('barrels')
    random_seed = LaunchConfiguration('random_seed')
    use_nav2 = LaunchConfiguration('use_nav2')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    headless = LaunchConfiguration('headless')
    limit_real_time_factor = LaunchConfiguration('limit_real_time_factor')
    wait_for_barrels = LaunchConfiguration('wait_for_barrels')
    gazebo_verbose = LaunchConfiguration('gazebo_verbose')

    # Vision sensor parameters
    vision_sensor = LaunchConfiguration('vision_sensor')
    vision_sensor_skip_frames = LaunchConfiguration('vision_sensor_skip_frames')
    vision_sensor_debug = LaunchConfiguration('vision_sensor_debug')
    vision_sensor_frame_divider = LaunchConfiguration('vision_sensor_frame_divider')

    # Initial position configuration for robots
    initial_pose_package = LaunchConfiguration('initial_pose_package')
    initial_pose_file = LaunchConfiguration('initial_pose_file')

    declare_initial_pose_file = DeclareLaunchArgument(
        'initial_pose_file',
        default_value='config/initial_poses.yaml',
        description="Location of initial pose yaml file relative to the package in 'initial_pose_package'")

    declare_initial_pose_package = DeclareLaunchArgument(
        'initial_pose_package',
        default_value='assessment',
        description="Package name for finding the file 'config/initial_poses.yaml'")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value="",
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('assessment'), 'params', 'nav2_params_namespaced.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_visualise_sensors_cmd = DeclareLaunchArgument(
        'visualise_sensors',
        default_value='false',
        description='Whether to visualise sensors in Gazebo')
    
    declare_odometry_source_cmd = DeclareLaunchArgument(
        'odometry_source',
        default_value='ENCODER',
        description='Odometry source - ENCODER or WORLD')
    
    declare_sensor_noise_cmd = DeclareLaunchArgument(
        'sensor_noise',
        default_value='false',
        description='Whether to enable sensor noise (applies to camera, LiDAR, and IMU)')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file to use')
    
    declare_rviz_windows_cmd = DeclareLaunchArgument(
        'rviz_windows',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'rviz_windows.yaml']),
        description='Full path to the RViz windows YAML file to use')

    declare_rviz_log_level_cmd = DeclareLaunchArgument(
        'rviz_log_level',
        default_value='warn',
        description='What level of messages from RViz to log')      
    
    declare_obstacles_cmd = DeclareLaunchArgument(
        'obstacles',
        default_value='true',
        description='Whether the world contains obstacles')

    declare_barrel_manager_cmd = DeclareLaunchArgument(
        'barrels',
        default_value='true',
        description='Whether to start the barrel manager')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for barrel manager')
    
    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Whether to use the navigation stack (Nav2)')
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run the Gazebo GUI')
    
    declare_limit_real_time_factor_cmd = DeclareLaunchArgument(
        'limit_real_time_factor',
        default_value='True',
        description='Whether to limit the Gazebo real-time factor to 1.0')
    
    declare_wait_for_barrels_cmd = DeclareLaunchArgument(
        'wait_for_barrels',
        default_value='False',
        description='Whether to wait for every barrel to spawn before spawning any robots')

    declare_vision_sensor_cmd = DeclareLaunchArgument(
        'vision_sensor',
        default_value='True',
        description='Whether to enable the vision sensor')

    declare_vision_sensor_skip_frames_cmd = DeclareLaunchArgument(
        'vision_sensor_skip_frames',
        default_value='True',
        description='Whether to skip processing of similar frames in vision_sensor detector')

    declare_vision_sensor_debug_cmd = DeclareLaunchArgument(
        'vision_sensor_debug',
        default_value='False',
        description='Whether to enable debug output imaging from vision_sensor detector')
    
    declare_vision_sensor_frame_divider_cmd = DeclareLaunchArgument(
        'vision_sensor_frame_divider',
        default_value='0.5',
        description='Number of frames to divide by when choosing how many frames to process. If value is 1.0, no division will occur.')

    declare_gazebo_verbose_cmd = DeclareLaunchArgument(
        'gazebo_verbose',
        default_value='False',
        description='Whether to enable verbose output from gazebo on the command line')

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

    # start_tf_relay_cmd = Node(
    #     package='tf_relay',
    #     executable='relay',
    #     output='screen',
    #     arguments=['robot', num_robots]
    # )

    start_barrel_manager_cmd = Node(
        package=package_name,
        executable='barrel_manager.py',
        output='screen',
        condition=IfCondition(barrel_manager),
        parameters=[{'random_seed' : random_seed}]
    )

    bringup_cmd_group = OpaqueFunction(function=group_action)
        
    ld = LaunchDescription()

    ld.add_action(SetUseSimTime(True))

    # Declare the launch options
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_visualise_sensors_cmd)
    ld.add_action(declare_odometry_source_cmd)
    ld.add_action(declare_sensor_noise_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_windows_cmd)
    ld.add_action(declare_obstacles_cmd)
    ld.add_action(declare_barrel_manager_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_limit_real_time_factor_cmd)
    ld.add_action(declare_wait_for_barrels_cmd)
    ld.add_action(declare_gazebo_verbose_cmd)
    ld.add_action(declare_rviz_log_level_cmd)

    # Vision sensor parameters
    ld.add_action(declare_vision_sensor_cmd)
    ld.add_action(declare_vision_sensor_skip_frames_cmd)
    ld.add_action(declare_vision_sensor_debug_cmd)
    ld.add_action(declare_vision_sensor_frame_divider_cmd)

    # Add launch initial pose option
    ld.add_action(declare_initial_pose_package)
    ld.add_action(declare_initial_pose_file)

    # Add the commands to the launch description
    ld.add_action(env_gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    #ld.add_action(start_tf_relay_cmd)
    ld.add_action(start_barrel_manager_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
