from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():   # This function needs this exact name

    robot_arg = LaunchConfiguration('robot')

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


    navigation_node = Node(
        package='project4b', 
        executable='navigation',
        output='screen',
        parameters=[
            { 'robot': robot_arg }
        ]
    )

    translator_node = Node(
        package='project4b', 
        executable='translator',
        output='screen',
        parameters=[
            { 'robot': robot_arg }
        ]
    )


    ld = LaunchDescription([
        robot_descriptor, 
        world_descriptor,
        navigation_node,
        translator_node, 
    ])

    return ld



