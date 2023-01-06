from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    arm_ns_arg = DeclareLaunchArgument(
        'arm_ns', default_value=TextSubstitution(text="utarm")
    )

    arm_ip_arg = DeclareLaunchArgument(
        'arm_ip', default_value=TextSubstitution(text="192.168.1.1")
    )

    report_hz_arg = DeclareLaunchArgument(
        'report_hz', default_value=TextSubstitution(text="10")
    )

    return LaunchDescription([
        arm_ns_arg,
        arm_ip_arg,
        report_hz_arg,


        Node(
            package='arm_controller',
            executable='utarm_report_publisher',
            name='utarm_report_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'arm_ns': LaunchConfiguration('arm_ns'),
                'arm_ip': LaunchConfiguration('arm_ip'),
                'report_hz': LaunchConfiguration('report_hz'),
            }]
        ),

        Node(
            package='arm_controller',
            executable='utarm_api_server',
            name='utarm_api_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'arm_ns': LaunchConfiguration('arm_ns'),
                'arm_ip': LaunchConfiguration('arm_ip'),
            }]
        ),
    ])
