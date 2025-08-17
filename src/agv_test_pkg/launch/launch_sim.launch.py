import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    # Gói chứa file launch robot_state_psublisher
    package_name = 'agv_test_pkg'  # <--- Đảm bảo đúng tên gói 
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch file robot_state_publisher
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description', 'urdf', 'agv_assem.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}


    # Launch Gazeboc
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
    )

    python_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('agv_test_pkg'), 'launch', 'launch_python_file.launch.py')
        ]),
    )

    lidar_a1m8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'view_sllidar_a1_launch.py')
        ]),
    )

    # comment
    # comment 2
    # 10 dong comment
     # 10 dong comment # 10 dong comment
      # 10 dong comment
       # 10 dong comment
        # 10 dong comment
         # 10 dong comment
          # 10 dong comment

    node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[params]
    )

    # Spawner entity vào Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'agv_test_pkg'],
        output='screen'
    )

    node_tf_map =Node( package="tf2_ros",
                            executable="static_transform_publisher",
                            output="screen" ,
                            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
                            )
    
    # Node mở RViz2
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'agv.rviz'  
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Launch tất cả node
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        node_robot_state_publisher,
        # node_tf_map,
        python_file,
        lidar_a1m8,
        rviz_node,
    ])
