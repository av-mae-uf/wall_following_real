from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("wall_following_real")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_following_real',
            executable='calc_steer',
            name='calc_steer',
            output='screen',
            parameters = [
                {'theta_deg': 25.0},  # angle from perp beam to beam 2, deg
                {'d_LookAhead': 3.5}, # distance to look-ahead point, m
                {'d_desired': 0.5},  # desired distance from wall, m
                {'speed': 0.3}, # m/sec
                {'Kp': 1.4},
                {'Ki': 0.05},
                {'Kd': 0.25},
                {'starting_delay': 5.0}, # seconds
                {'save_to_file': True},
                {'file_name': 'real_wall.csv'},
            ] 
        ),
    ])