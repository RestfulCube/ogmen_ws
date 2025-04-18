import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch import LaunchDescription
import os

def generate_launch_description():
    # Constants for paths to different files and folders
    package_name = 'bot_description'
    robot_name_in_model = 'my_bot'
    robot_model_path = 'urdf/robot.urdf.xacro'
    
    # Pose where to spawn the robot 
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.2'  # Higher elevation to prevent falling through ground
    spawn_roll_val = '0.00'
    spawn_pitch_val = '0.00'
    spawn_yaw_val = '0.00'

    # Set path to files and directories
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_robot_model_path = os.path.join(pkg_share, robot_model_path)
    
    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration('urdf_model')

    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_model_path = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_robot_model_path, 
        description='Absolute path to robot urdf file')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.  
    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', urdf_model])}])

    # Publish the joint states of the robot
    start_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])

    # Launch Ignition Gazebo with default empty world
    start_gazebo_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r'],
        output='screen'
    )

    # Spawn robot into Gazebo Ignition
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_in_model,
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-R', spawn_roll_val,
            '-P', spawn_pitch_val,
            '-Y', spawn_yaw_val
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
 
    # Declare the launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_model_path)
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_joint_state_publisher_node)
    ld.add_action(start_gazebo_sim)
    ld.add_action(spawn_entity)
    
    return ld