from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from project4a_v2.load_robot import load_disc_robot



def generate_launch_description():   # This function needs this exact name

    robot_arg = LaunchConfiguration('robot')
    world_arg = LaunchConfiguration('world')


    robot = load_disc_robot('normal.robot.txt')

    robot_state_publisher_node = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        parameters=[
            {'robot_description': robot['urdf']}
        ],
        output='screen'
    )

    robot_descriptor = DeclareLaunchArgument(
        'robot',
        default_value='normal.robot.txt', 
        description='Robot argument'
    )
    world_descriptor = DeclareLaunchArgument(
        'world',
        default_value='brick.world.txt', 
        description='World argument'
    )

    jimmy_node = Node(
        package='project4a_v2',
        executable='jimmy', 
        parameters=[
            { 'robot': robot_arg, 'world': world_arg }
        ]
    )

    ld = LaunchDescription([
        robot_descriptor, 
        world_descriptor,
        jimmy_node,
        robot_state_publisher_node,
    ])

    return ld



