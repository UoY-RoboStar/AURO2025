import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import yaml

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, Shutdown, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace, RosTimer, SetUseSimTime


def robot_controller_actions(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
        
    # If using different initial poses, change them in the provided file.
    yaml_path = os.path.join(context.launch_configurations['initial_pose_package'], context.launch_configurations['initial_pose_file'])

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    if num_robots > 0:
        initial_poses = configuration[num_robots]
    else:
        initial_poses = {}

    actions = []

    for robot_number in range(1, num_robots + 1):

        robot_name = 'robot' + str(robot_number)

        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            Node(
                package='solution',
                executable='robot_controller',
                #
                # You can uncomment the 'prefix' option below to open the log for this node in a separate window,
                # that will be displayed in the current desktop, ie. for auro-vnc within VNC, and for auro-wsl in the Windows desktop.
                #
                # prefix=['lxterminal -e'], # Opens in new window, should work under auro-vnc, auro-wsl or X11 native.
                output='screen',
                # output='log', # Instead use to log to a file under ~/.ros/log/.
                parameters=[initial_poses[robot_name]]),

            # Node(
            #     package='turtlebot3_gazebo',
            #     executable='turtlebot3_drive',
            #     output='screen'),

        ])

        actions.append(group)

    return actions

def generate_launch_description():

    package_name = 'solution'

    num_robots = LaunchConfiguration('num_robots')
    random_seed = LaunchConfiguration('random_seed')
    experiment_duration = LaunchConfiguration('experiment_duration')
    data_log_path = LaunchConfiguration('data_log_path')
    data_log_filename = LaunchConfiguration('data_log_filename')
    obstacles = LaunchConfiguration('obstacles')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    sensor_noise = LaunchConfiguration('sensor_noise')
    visualise_sensors = LaunchConfiguration('visualise_sensors')
    odometry_source = LaunchConfiguration('odometry_source')
    barrel_manager = LaunchConfiguration('barrels')
    headless = LaunchConfiguration('headless')
    limit_real_time_factor = LaunchConfiguration('limit_real_time_factor')
    wait_for_barrels = LaunchConfiguration('wait_for_barrels')
    gazebo_verbose = LaunchConfiguration('gazebo_verbose')

    vision_sensor_skip_frames = LaunchConfiguration('vision_sensor_skip_frames')
    vision_sensor_frame_divider = LaunchConfiguration('vision_sensor_frame_divider')
    vision_sensor_debug = LaunchConfiguration('vision_sensor_debug')

    initial_pose_package = LaunchConfiguration('initial_pose_package')
    initial_pose_file = LaunchConfiguration('initial_pose_file')
   
    declare_data_log_path_cmd = DeclareLaunchArgument(
        'data_log_path',
        default_value = os.path.join(get_package_prefix(package_name), '../../'),
        description='Full path to directory where data logs will be saved')
    
    declare_data_log_filename_cmd = DeclareLaunchArgument(
        'data_log_filename',
        default_value='data_log',
        description='Filename prefix to use for data logs')

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Whether to use nav2'
    )

    # Parameters controlling launch of the barrels
    declare_barrel_manager_cmd = DeclareLaunchArgument(
        'barrels',
        default_value='True',
        description='Whether to launch barrels'
    )

    declare_wait_for_barrels_cmd = DeclareLaunchArgument(
        'wait_for_barrels',
        default_value='True',
        description='Whether to wait for barrels to be spawn before spawing the turtlebots'
    )    

    # Parameters for the vision sensors of every robot
    declare_visual_sensor_cmd = DeclareLaunchArgument(
        'visual_sensor',
        default_value='True',
        description='Whether to launch the visual_sensor for every robot'
    )

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

    # Parameters for the simulation
    declare_obstacles_cmd = DeclareLaunchArgument(
        'obstacles',
        default_value='true',
        description='Whether the world contains obstacles')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for barrel manager')
    
    declare_experiment_duration_cmd = DeclareLaunchArgument(
        'experiment_duration',
        default_value='840.0',
        description='Experiment duration in seconds of simulation time')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz2'
    )

    declare_rviz_log_level_cmd = DeclareLaunchArgument(
        'rviz_log_level',
        default_value='warn',
        description='Log level for RViz2'
    )

    declare_sensor_noise_cmd = DeclareLaunchArgument(
        'sensor_noise',
        default_value='False',
        description='Whether to include sensor noise'
    )

    declare_visualise_sensors_cmd = DeclareLaunchArgument(
        'visualise_sensors',
        default_value='false',
        description='Whether to visualise sensors in the simulation (e.g. LiDAR)'
    )

    declare_odometry_source_cmd = DeclareLaunchArgument(
        'odometry_source',
        default_value='ENCODER',
        description='Selecting the source for odometry: ENCODER for wheel encoder, or WORLD for idealised data'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to launch the simulation headless'
    )

    declare_limit_real_time_factor_cmd = DeclareLaunchArgument(
        'limit_real_time_factor',
        default_value='True',
        description='Whether to limit simulation to execute in real time'
    )
    
    declare_gazebo_verbose_cmd = DeclareLaunchArgument(
        'gazebo_verbose',
        default_value='False',
        description='Whether to enable verbose output from gazebo on the command line'
    )

    # Parameters for configuring the initial pose of the robots
    declare_initial_pose_file_cmd = DeclareLaunchArgument(
        'initial_pose_file',
        default_value='config/initial_poses.yaml',
        description="Location of initial pose yaml file relative to the package in 'initial_pose_package'"
    )

    declare_initial_pose_package_cmd = DeclareLaunchArgument(
        'initial_pose_package',
        default_value='assessment',
        description="Package name for finding the file 'config/initial_poses.yaml'"
    )

    # RViz2 configuration, which you can change if needed, by pointing it to your solution package,
    # rather than using the files under the assessment package.
    rviz_config = PathJoinSubstitution([FindPackageShare('assessment'), 'rviz', 'namespaced.rviz'])
    rviz_windows = PathJoinSubstitution([FindPackageShare('assessment'), 'config', 'rviz_windows.yaml'])
    
    # Nav2 parameters. Similarly, can be changed by pointing it to your solution package.
    map = PathJoinSubstitution([FindPackageShare('solution'), 'config', 'map2.yaml'])
    params = PathJoinSubstitution([FindPackageShare('assessment'), 'params', 'nav2_params_namespaced.yaml'])
    
    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment.launch.py'
                ])
        ),
        launch_arguments={'num_robots': num_robots,
                          'visualise_sensors': visualise_sensors,
                          'odometry_source': odometry_source,
                          'sensor_noise': sensor_noise,
                          'use_rviz': use_rviz,
                          # 'rviz_config': rviz_config, # only change if needed, as default is picked up automatically
                          # 'rviz_windows': rviz_windows, # only change if needed
                          'barrels': barrel_manager,
                          'random_seed': random_seed,
                          'use_nav2': use_nav2,
                          'map': map,
                          'params_files': params,
                          'headless': headless,
                          'obstacles': obstacles,
                          'limit_real_time_factor': limit_real_time_factor,
                          'wait_for_barrels': wait_for_barrels,
                          'initial_pose_file': initial_pose_file,
                          'initial_pose_package': initial_pose_package,
                          'gazebo_verbose': gazebo_verbose,
                          'rviz_log_level': rviz_log_level
                          }.items()
    )

    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    data_logger_cmd = Node(
        package='solution',
        executable='data_logger',
        output='screen',
        arguments=['--path', data_log_path,
                   '--filename', data_log_filename,
                   '--random_seed', random_seed])

    timeout_cmd = RosTimer(                                         
            period = experiment_duration,
            actions = [                                                       
                Shutdown(reason="Experiment timeout reached")     
            ],
        )

    ld = LaunchDescription()

    # Do not remove
    ld.add_action(SetUseSimTime(True))

    ld.add_action(declare_obstacles_cmd)
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_experiment_duration_cmd)
    ld.add_action(declare_data_log_path_cmd)
    ld.add_action(declare_data_log_filename_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_sensor_noise_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_visualise_sensors_cmd)
    ld.add_action(declare_odometry_source_cmd)
    ld.add_action(declare_barrel_manager_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_limit_real_time_factor_cmd)
    ld.add_action(declare_wait_for_barrels_cmd)
    ld.add_action(declare_vision_sensor_skip_frames_cmd)
    ld.add_action(declare_vision_sensor_frame_divider_cmd)
    ld.add_action(declare_vision_sensor_debug_cmd)
    ld.add_action(declare_initial_pose_file_cmd)
    ld.add_action(declare_initial_pose_package_cmd)
    ld.add_action(declare_gazebo_verbose_cmd)
    ld.add_action(declare_rviz_log_level_cmd)

    ld.add_action(assessment_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(data_logger_cmd)
    ld.add_action(timeout_cmd)    

    return ld
