#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from project4b.load_robot import load_disc_robot


def controller(data): 
    v = data.linear.x 
    w = data.angular.z
    l = robot['wheels']['distance']

    vl, vr = Float64(), Float64()
    vl.data = v - w*(l/2)
    vr.data = v + w*(l/2)

    vl_publisher.publish(vl)
    vr_publisher.publish(vr)



def main(args=None):
    global vl_publisher
    global vr_publisher
    global robot
    global node
    

    rclpy.init(args=args)

    # Node creation
    node = rclpy.create_node('translation_controller')

    # Getting the name of which robot to be used and loading it
    node.declare_parameter('robot', 'normal.robot.txt')
    robot = load_disc_robot(node.get_parameter('robot').value)

    # Create the publisher and subscriptions
    vl_publisher = node.create_publisher(Float64, '/vl', 10)
    vr_publisher = node.create_publisher(Float64, '/vr', 10)
    node.create_subscription(Twist, '/cmd_vel', controller, 10)

    # Complete the node
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
