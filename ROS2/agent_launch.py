from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['sudo', 'socat', 'pty,link=/dev/virtualcom0,raw,group-late=dialout,mode=660', 'udp-listen:5000'], shell=True, output='screen'
        ),
        ExecuteProcess(
            cmd=['sudo', 'socat', 'pty,link=/dev/virtualcom1,raw,group-late=dialout,mode=660', 'udp-listen:5001'], shell=True, output='screen'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            emulate_tty=True,
            arguments=['multiserial', '--devs',
                       "/dev/virtualcom0 /dev/virtualcom1", 'v6']
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            emulate_tty=True,
            arguments=['udp4', '--port', '8888', 'v6']
        )
    ])
