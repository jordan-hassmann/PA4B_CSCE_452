#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from project4a_v2.load_robot import load_disc_robot
from math import sin, cos, atan2, pi


angle_min, angle_max = -1.8, 1.8
angle_inc = (angle_max - angle_min) / 25
V_MAX, W_MAX = 2, 2
publisher = None 



def get_direction(ranges): 

    weighted_x = weighted_y = 0 

    for r, θ in zip(ranges, range(angle_min, angle_inc, angle_max+angle_inc)): 
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
    rclpy.init(args=args)

    node = rclpy.create_node('navigation_controller')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    node.create_subscription(LaserScan, '/scan', controller, 10)

    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
