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
    urdf_file_name = 'model/model.sdf'
    urdf = os.path.join(
        get_package_share_directory('physics'),
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
        
        # Your custom controller node
        Node(
            package='controller',
            executable='controller',
            name='controller',
            output='screen'
        ),
        
        # Parameter bridge node for ROS 2 to Gazebo communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(get_package_share_directory('config_files'), 'config/bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local'
            }],
            output='screen'
        )
    ])