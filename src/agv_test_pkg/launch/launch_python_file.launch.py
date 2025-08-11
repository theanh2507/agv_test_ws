import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Node cho odom_tf_pub.py
    odom_tf2 = Node(
        package='agv_test_pkg',                  
        executable='odom_tf_pub.py',             
        name='odom_tf_publisher',            
        output='screen'                 
    )

    # Node cho pub_joint_state.py
    # publish_joint_state = Node(
    #     package='agv_test_pkg',
    #     executable='pub_joint_state.py',
    #     name='joint_state_publisher',          
    #     output='screen'
    # )

    # Node cho caculate_position.py
    caculate_pos = Node(
        package='agv_test_pkg',
        executable='caculate_position.py',
        name='modbus_rtu_node',            
        output='screen'
    )

    # control_and_calculate = Node(
    #     package='agv_test_pkg',
    #     executable='control_calculate.py',
    #     name='control_and_calculate',            
    #     output='screen'
    # )

    lidar_angel = Node(
        package='agv_test_pkg',
        executable='lidar.py',
        name='lidar_angel',
        output='screen'
    )

    cmd_vel_mux = Node(
        package='agv_test_pkg',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen'
    )


    multiple_point = Node(
        package='agv_test_pkg',
        executable='multiple_point.py',
        name='multiple_point',
        output='screen'
    )

    waypoint = Node(
        package='agv_test_pkg',
        executable='waypoint.py',
        name='waypoint',
        output='screen'
    )

    ld.add_action(odom_tf2)
    ld.add_action(lidar_angel)
    ld.add_action(caculate_pos)
    ld.add_action(cmd_vel_mux)
    ld.add_action(multiple_point)
    ld.add_action(waypoint)

    return ld