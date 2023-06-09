#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from project4b.load_robot import load_disc_robot
from math import sin, cos, atan2, pi, isnan, isinf



def get_direction(ranges): 
    ANGLE_MIN = robot['laser']['angle_min']
    ANGLE_MAX = robot['laser']['angle_max']
    ANGLE_INC = (ANGLE_MAX - ANGLE_MIN) / robot['laser']['count']

    x = y = 0
    n = 0

    for i, r in enumerate(ranges): 
        if isnan(r) or isinf(r): continue

        θ = ANGLE_MIN + i * ANGLE_INC
        x += r*cos(θ)
        y += r*sin(θ)
        n += 1

    x /= n 
    y /= n 

    return x, y


def get_velocities(x, y): 

    θ = atan2(y, x)
    v = ANGLE_MIN * V_MAX * (pi/ANGLE_MIN - abs(θ)) / pi
    w = W_MAX * θ / pi

    if v < 0 or x < 0.3: 
        v = 0.0
        w = 0.5
    
    return v, w
    
def controller(data): 

    x, y = get_direction(data.ranges)
    v, w = get_velocities(x, y)

    t = Twist() 
    t.linear.x = v 
    t.angular.z = w

    publisher.publish(t)
    

def main(args=None):
    global publisher 
    global robot
    global node
    global W_MAX 
    global V_MAX
    global ANGLE_MIN


    rclpy.init(args=args)

    # Node creation
    node = rclpy.create_node('navigation_controller')

    # Declare parameters
    node.declare_parameter('robot', 'normal.robot.txt')
    node.declare_parameter('v_max', 0.1)
    node.declare_parameter('w_max', 2.5)
    node.declare_parameter('angle_min', 7)
    robot = load_disc_robot(node.get_parameter('robot').value)
    V_MAX = node.get_parameter('v_max').value
    W_MAX = node.get_parameter('w_max').value
    ANGLE_MIN = node.get_parameter('angle_min').value

    # Create the publisher and subscriptions
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    node.create_subscription(LaserScan, '/scan', controller, 10)

    # Complete the node
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
