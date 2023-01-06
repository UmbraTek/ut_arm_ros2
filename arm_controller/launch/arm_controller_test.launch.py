from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    arm_ns_arg = DeclareLaunchArgument(
        'arm_ns', default_value=TextSubstitution(text="utarm")
    )

    test_type_arg = DeclareLaunchArgument(
        'test_type', default_value=TextSubstitution(text="type1")
    )

    return LaunchDescription([
        arm_ns_arg,
        test_type_arg,
        Node(
            package='arm_controller',
            executable='arm_controller_test',
            name='arm_controller_test',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'arm_ns': LaunchConfiguration('arm_ns'),
                'test_type': LaunchConfiguration('test_type'),
            }]
        ),
    ])
