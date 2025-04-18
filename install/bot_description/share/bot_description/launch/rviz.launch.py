# import os
# from launch import LaunchDescription
# from launch.substitutions import Command
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # # Get the package directory
    # pkg_dir = get_package_share_directory('bot_description')
    
    # # Path to the xacro file
    # xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    
    # # Get the robot description from xacro
    # robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'robot_description': robot_description},
    #         {'use_sim_time': True},
    #     ])
    
    # Joint State Publisher
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    # )
    
    # RViz
    # rviz_config = os.path.join(pkg_dir, 'rviz', 'urdf_config.rviz')
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config]
    # )
    
    # return LaunchDescription([
    #     robot_state_publisher,
    #     # joint_state_publisher,
    #     # rviz
    # ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'bot_description',
                'urdf_package_path': PathJoinSubstitution(['urdf', 'robot.urdf.xacro'])
            }.items()
        )
    ])