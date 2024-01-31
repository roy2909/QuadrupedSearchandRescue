

"""
Launches rviz with the go1 urdf file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration, \
     IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, \
     TextSubstitution
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

gazebo_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

def generate_launch_description():
     return LaunchDescription([
          DeclareLaunchArgument(name='use_jsp', default_value='jsp',
                                choices=['gui', 'jsp', 'none'],
                                description='Choose if joint_state_publisher is launched'),
          DeclareLaunchArgument(name='use_rviz', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if rviz is launched'),
          DeclareLaunchArgument(name='use_nav2_links', default_value='false',
                                choices=['true', 'false'],
                                description='Use Nav2 frames in URDF'),
          DeclareLaunchArgument(name='fixed_frame', default_value='base_link',
                                description='Fixed frame for RVIZ'),
          DeclareLaunchArgument(name='namespace', default_value='',
                                description=
                                   'Choose a namespace for the launched topics.'),

          SetLaunchConfiguration(name='config_file',
                                 value='go1.rviz'),
          SetLaunchConfiguration(name='model',
                                 value=PathJoinSubstitution([FindPackageShare('go1_description'),
                                                             'xacro',
                                                             'robot.xacro'])),
          SetLaunchConfiguration(name='rvizconfig',
                                 value=PathJoinSubstitution([FindPackageShare('go1_description'),
                                                             'config',
                                                             LaunchConfiguration('config_file')])),
          SetLaunchConfiguration(name='gazebo_path',
                                 value=PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                                             'launch',
                                                             "gazebo.launch.py"])),

          Node(package='joint_state_publisher',
               executable='joint_state_publisher',
               namespace=LaunchConfiguration('namespace'),
               condition=LaunchConfigurationEquals('use_jsp', 'none')),

          Node(package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               namespace=LaunchConfiguration('namespace'),
               condition=LaunchConfigurationEquals('use_jsp', 'gui')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_path),
            launch_arguments={
                'use_sim_time': str("true"),
            }.items()
        ),

          Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{
                    'robot_description':
                         ParameterValue(
                              Command([
                                   'xacro ',
                                   LaunchConfiguration('model'),
                                   TextSubstitution(text=' use_nav2_links:='),
                                   LaunchConfiguration('use_nav2_links'),
                              ]),
                              value_type=str
                         ),
                    'frame_prefix': [LaunchConfiguration('namespace'), '/']
               }],
               namespace=LaunchConfiguration('namespace')
          ),

           Node(package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic',{
                    'robot_description':
                         ParameterValue(
                              Command([
                                   'xacro ',
                                   LaunchConfiguration('model'),
                                   TextSubstitution(text=' use_nav2_links:='),
                                   LaunchConfiguration('use_nav2_links'),
                              ]),
                              value_type=str
                         ), 
               },'-entity', 'go1_gazebo', '-z 0.1'],
               namespace=LaunchConfiguration('namespace'),
                output='screen'
          ),
        

          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rvizconfig'),
                    '-f', LaunchConfiguration('fixed_frame')
               ],
               condition=IfCondition(LaunchConfiguration('use_rviz')))
    ])