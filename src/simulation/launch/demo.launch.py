import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define the URDF file
    urdf_file_name = 'model.urdf'
    urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name
    )

    # Read the URDF file content
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Launch Gazebo simulation with the URDF model
        ExecuteProcess(
            cmd=['gz', 'sim', urdf],
            output='screen'
        ),
        # Robot State Publisher to publish the robot state based on the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
            arguments=[urdf]
        ),
        # Your custom controller node
        Node(
            package='simulation',
            executable='controller',
            name='actuation',
            output='screen'
        ),
        # Parameter bridge node for ROS 2 to Gazebo communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(get_package_share_directory('simulation'), 'bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local'
            }],
            output='screen'
        ),
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        # ROS 2 Control Node to manage controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[os.path.join(get_package_share_directory('simulation'), 'control.yaml')],
            output='screen'
        )
    ])