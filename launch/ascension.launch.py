from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('publish_tf', default_value='false', description='Publish TF transforms'),
        DeclareLaunchArgument('px', default_value='0.04', description='Pivot X'),
        DeclareLaunchArgument('py', default_value='0.0', description='Pivot Y'),
        DeclareLaunchArgument('pz', default_value='0.0', description='Pivot Z'),
        DeclareLaunchArgument('gx', default_value='0.12', description='Grab X'),
        DeclareLaunchArgument('gy', default_value='0.0', description='Grab Y'),
        DeclareLaunchArgument('gz', default_value='0.0', description='Grab Z'),
        DeclareLaunchArgument('use_grab_frame', default_value='true', description='Use grab frame'),
        
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
                'grab_x': LaunchConfiguration('gx'),
                'grab_y': LaunchConfiguration('gy'),
                'grab_z': LaunchConfiguration('gz'),
                'use_grab_frame': LaunchConfiguration('use_grab_frame'),
            }]
        ),
        
        # Note: phidgets nodes commented out - you'll need to install phidgets ROS2 package
        # and uncomment these if needed
        # Node(
        #     package='phidgets',
        #     executable='interface_kit',
        #     name='phidgets_interface_pinchers',
        #     output='screen',
        #     parameters=[{
        #         'serial': 265399,
        #     }]
        # ),
        
        # Node(
        #     package='phidgets',
        #     executable='interface_kit',
        #     name='phidgets_interface_clutch',
        #     output='screen',
        #     parameters=[{
        #         'serial': 270130,
        #     }]
        # ),
    ])

