#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from project4b.load_robot import load_disc_robot


vl_publisher = vr_publisher = None
l = 1


def controller(data): 
    v = data.linear.x 
    w = data.angular.z

    vl, vr = Float64(), Float64()
    vl.data = v - w*(l/2)
    vr.data = v + w*(l/2)

    vl_publisher.publish(vl)
    vr_publisher.publish(vr)



def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('navigation_controller')
    vl_publisher = node.create_publisher(Float64, '/vl', 10)
    vr_publisher = node.create_publisher(Float64, '/vr', 10)
    node.create_subscription(Twist, '/cmd_vel', controller, 10)

    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
