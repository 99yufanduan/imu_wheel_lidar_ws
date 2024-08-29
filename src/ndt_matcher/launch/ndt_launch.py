from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', 'src/ndt_matcher/config/ndt.rviz']
        ),
        Node(
            package='ndt_matcher',
            executable='ndt_matcher',
            name='ndt_matcher',
            output='screen',
        ),
        Node(
            package='imu_kalman',
            executable='my_kalman_node',
            name='my_kalman_node',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_odom_to_base_link',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'vanjee_lidar']
        )
    ])