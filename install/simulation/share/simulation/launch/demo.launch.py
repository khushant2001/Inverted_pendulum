import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
import launch
from launch.actions import ExecuteProcess

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config = get_package_share_directory('simulation')
    urdf_file_name = 'model.urdf'
    sdf_file_name = 'model.sdf'
    urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name)
    sdf = os.path.join(
        get_package_share_directory('simulation'),
        sdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', urdf],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='simulation',
            executable='controller',
            name='actuation',
            output='screen'),

        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(config,'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
        ),

        Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(config,'control.yaml')],
            output='screen'
            )
    ])