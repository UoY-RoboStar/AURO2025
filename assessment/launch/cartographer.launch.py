from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_rviz = LaunchConfiguration('use_rviz')

    ns = LaunchConfiguration('ns')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz2'
    )

    declare_ns = DeclareLaunchArgument(
        'ns',
        default_value='robot1',
        description='Namespace for all nodes'
    )

    return LaunchDescription([
            declare_use_sim_time,
            declare_use_rviz,
            declare_ns,
            PushRosNamespace(ns),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),
            LogInfo(msg=['use_rviz resolved to: ', use_rviz]),
            LogInfo(msg=['use_sim_time resolved to: ', use_sim_time]),   
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('assessment'), 'launch', 'rviz.launch.py')
                ),
                launch_arguments={'ros_namespace': ns, 'rviz_config': os.path.join(get_package_share_directory('assessment'), 'rviz', 'namespaced_cartographer.rviz')}.items(),
                condition=IfCondition(use_rviz)
            ),                     
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
                ),
                launch_arguments={'use_rviz': 'false', 'use_sim_time': use_sim_time, 'cartographer_config_dir': os.path.join(get_package_share_directory('assessment'), 'config')}.items()
            )
        ])