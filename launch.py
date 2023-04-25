from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():   # This function needs this exact name

    robot_arg = LaunchConfiguration('robot')
    linear_velocity_arg = LaunchConfiguration('v_max')
    angular_velocity_arg = LaunchConfiguration('w_max')
    min_velocity_angle_arg = LaunchConfiguration('angle_min')

    robot_descriptor = DeclareLaunchArgument(
        'robot',
        default_value='normal.robot.txt', 
        description='Robot argument'
    )

    linear_velocity_descriptor = DeclareLaunchArgument(
        'v_max', 
        default_value='0.1',
        description='The max LINEAR velocity of the robot'
    )

    angular_velocity_descriptor = DeclareLaunchArgument(
        'w_max',
        default_value='2.5',
        description='The max ANGULAR velocity of the robot'
    )

    min_velocity_angle_descriptor = DeclareLaunchArgument(
        'angle_min', 
        default_value='7',
        description='The demoninator for pi/n of which the linear velocity is tuned to 0',
    )


    navigation_node = Node(
        package='project4b', 
        executable='navigation',
        output='screen',
        parameters=[{ 
            'robot': robot_arg, 
            'v_max': linear_velocity_arg,
            'w_max': angular_velocity_arg,
            'angle_min': min_velocity_angle_arg
        }]
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
        linear_velocity_descriptor,
        angular_velocity_descriptor,
        min_velocity_angle_descriptor,
        navigation_node,
        translator_node, 
    ])

    return ld



