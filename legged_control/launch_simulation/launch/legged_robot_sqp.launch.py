import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type', default_value='b1', description='Robot type: [a1, go1, b1]'
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='xmlFile',
            default_value=PathJoinSubstitution([  # File path for the URDF
            get_package_share_directory('mujoco_simulator'),
            'models',
            LaunchConfiguration('robot_type'),
            'urdf',
            'robot.xml'
            ])
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=PathJoinSubstitution([  # File path for the URDF
            get_package_share_directory('mujoco_simulator'),
            'models',
            LaunchConfiguration('robot_type'),
            'urdf',
            'robot.urdf'
            ])
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='simulatorFile',
            default_value=PathJoinSubstitution([  # File path for the URDF
            get_package_share_directory('user_command'),
            'config',
            LaunchConfiguration('robot_type'),
            'simulation.info'
            ])
        ),
        
        
        launch.actions.DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=PathJoinSubstitution([  # File path for the gait info
            get_package_share_directory('user_command'),
            'config',
            LaunchConfiguration('robot_type'),
            'gait.info'
            ])
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='referenceFile',
            default_value=PathJoinSubstitution([  # File path for the reference info
            get_package_share_directory('user_command'),
            'config',
            LaunchConfiguration('robot_type'),
            'reference.info'
            ])
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=PathJoinSubstitution([  # File path for the task info
            get_package_share_directory('user_command'),
            'config',
            LaunchConfiguration('robot_type'),
            'task.info'
            ])
        ),
        
        # Define parameters for robot URDF and configuration files (unchanged)
        Node(
            package='mujoco_simulator',
            executable='mujoco_simulator',
            name='mujoco_simulator',
            output='screen',
            prefix="gnome-terminal --",
            parameters=[
            	{
                    'xmlFile': launch.substitutions.LaunchConfiguration('xmlFile')
                },
                {
                    'simulatorFile': launch.substitutions.LaunchConfiguration('simulatorFile')
                }
            ],
        ),
        
        Node(
            package='user_command',
            executable='user_command_node',
            name='user_command_node',
            output='screen',
            prefix="gnome-terminal --",
            parameters=[
            	{
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ],
        ),
        
        
        # Add a TimerAction to wait 5 seconds before starting the next node
        TimerAction(
            period=5.0,  # Wait for 5 seconds
            actions=[Node(
            package='motion_control',
            executable='legged_robot_sqp_mpc',
            name='legged_robot_sqp_mpc',
            output='screen',
            prefix="gnome-terminal --",
            parameters=[
            	{
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'simulatorFile': launch.substitutions.LaunchConfiguration('simulatorFile')
                }
                ],
        )]
        ),
        
        
    ])

