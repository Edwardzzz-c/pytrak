from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('publish_tf', default_value='true', description='Publish TF transforms'),
        DeclareLaunchArgument('px', default_value='0.0', description='Pivot X for sensor 0'),
        DeclareLaunchArgument('py', default_value='0.0', description='Pivot Y for sensor 0'),
        DeclareLaunchArgument('pz', default_value='0.0', description='Pivot Z for sensor 0'),
        DeclareLaunchArgument('rx', default_value='0.0', description='Attach roll for sensor 0'),
        DeclareLaunchArgument('ry', default_value='0.0', description='Attach pitch for sensor 0'),
        DeclareLaunchArgument('rz', default_value='0.0', description='Attach yaw for sensor 0'),
        
        DeclareLaunchArgument('px1', default_value='0.0', description='Pivot X for sensor 1'),
        DeclareLaunchArgument('py1', default_value='0.0', description='Pivot Y for sensor 1'),
        DeclareLaunchArgument('pz1', default_value='0.0', description='Pivot Z for sensor 1'),
        DeclareLaunchArgument('rx1', default_value='0.0', description='Attach roll for sensor 1'),
        DeclareLaunchArgument('ry1', default_value='0.0', description='Attach pitch for sensor 1'),
        DeclareLaunchArgument('rz1', default_value='0.0', description='Attach yaw for sensor 1'),
        
        DeclareLaunchArgument('use_grab_frame', default_value='true', description='Use grab frame'),
        DeclareLaunchArgument('hemisphere_back', default_value='false', description='Use back hemisphere'),
        DeclareLaunchArgument('range_72inch', default_value='false', description='Use 72 inch range'),
        
        # Trakstar driver node
        Node(
            package='trakstar',
            executable='trakstar_node',
            name='trakstar_driver',
            output='screen',
            parameters=[{
                'publish_tf': LaunchConfiguration('publish_tf'),
                'pivot_x': LaunchConfiguration('px'),
                'pivot_y': LaunchConfiguration('py'),
                'pivot_z': LaunchConfiguration('pz'),
                'attach_roll': LaunchConfiguration('rx'),
                'attach_pitch': LaunchConfiguration('ry'),
                'attach_yaw': LaunchConfiguration('rz'),
                'pivot_x1': LaunchConfiguration('px1'),
                'pivot_y1': LaunchConfiguration('py1'),
                'pivot_z1': LaunchConfiguration('pz1'),
                'attach_roll1': LaunchConfiguration('rx1'),
                'attach_pitch1': LaunchConfiguration('ry1'),
                'attach_yaw1': LaunchConfiguration('rz1'),
                'use_grab_frame': LaunchConfiguration('use_grab_frame'),
                'hemisphere_back': LaunchConfiguration('hemisphere_back'),
                'range_72inch': LaunchConfiguration('range_72inch'),
                'frequency': 80,
            }]
        ),
    ])

