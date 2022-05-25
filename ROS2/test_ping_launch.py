from launch import LaunchDescription
from launch_ros.actions import Node
#from tracetools_launch.action import Trace


def generate_launch_description():
    return LaunchDescription([
        #Trace(
        #    session_name='test_ping_trace',
        #    base_path='/home/user/pnn16004/ros2_ws/tracing/',
        #    events_kernel=[]
        #),
        Node(
            package='transmission',
            executable='test_ping',
            output='screen',
            parameters=[
                {"my_ms": 100}
            ],
            remappings=[
            	("status", "/myumi_000/platform/joint_states")
            ],
        ),
    ])

"""
Create a Trace.

:param session_name: the name of the tracing session
:param append_timestamp: whether to append timestamp to the session name
:param base_path: the path to the base directory in which to create the session directory, or `None` for default
:param events_ust: the list of ROS UST events to enable
:param events_kernel: the list of kernel events to enable
:param context_names: the list of context names to enable
:param profile_fast: `True` to use fast profiling, `False` for normal (only if necessary)
"""
