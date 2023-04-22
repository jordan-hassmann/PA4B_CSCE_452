#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from project4b.load_robot import load_disc_robot
from math import sin, cos, atan2, pi


V_MAX, W_MAX = 2, 2


def get_direction(ranges): 
    ANGLE_MIN = robot['laser']['angle_min']
    ANGLE_MAX = robot['laser']['angle_max']
    ANGLE_INC = (ANGLE_MAX - ANGLE_MIN) / robot['laser']['count']

    # Get the weighted sums for each range in cartesian form, and return direction of resulting vector
    weighted_x = weighted_y = 0 

    for r, θ in zip(ranges, range(ANGLE_MIN, ANGLE_INC, ANGLE_MAX+ANGLE_INC)): 
        x, y = r*cos(θ), r*sin(θ)
        weighted_x += r*x 
        weighted_y += r*y

    return atan2(weighted_y, weighted_x)

def get_velocities(θ): 
    v = V_MAX * (pi - abs(θ)) / pi
    w = W_MAX * θ / pi
    return v, w
    
def controller(data): 
    θ = get_direction(data.ranges)
    v, w = get_velocities(θ)
    
    t = Twist() 
    t.linear.x = v 
    t.angular.z = w

    publisher.publish(t)
    

def main(args=None):
    global publisher 
    global robot


    rclpy.init(args=args)

    # Node creation
    node = rclpy.create_node('navigation_controller')

    # Getting the name of which robot to be used and loading it
    node.declare_parameter('robot', 'normal.robot.txt')
    robot = load_disc_robot(node.get_parameter('robot').value)

    # Create the publisher and subscriptions
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    node.create_subscription(LaserScan, '/scan', controller, 10)

    # Complete the node
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
