import launch
from ament_index_python.packages import get_package_share_directory
from launch import action
from launch import actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

def generate_launch_description():
    
    # extracting the robot definition from the xacro file
    franka_xacro_file = os.path.join(get_package_share_directory('boxer_description'), 'urdf',
                                     'boxer.urdf.xacro')
    robot_description_content = xacro.process_file(franka_xacro_file).toxml()


    # Create the launch configuration variables
    #use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{#'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
            }]
    )

    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'boxer',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0',
            '-z', '0.01'
        ],
        output='screen'
    )

    diff_drive_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_drive_controller"],
    )

    joint_broad_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
)

    
    return launch.LaunchDescription([
        #launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            #description='Use simulation (Gazebo) clock if true'),
        #launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            #description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),

        robot_state_publisher_node,
        spawn_entity,
        diff_drive_spawner, 
        joint_broad_spawner
    ])