#!/usr/bin/env python3

import rclpy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile




from project4b.load_robot import load_disc_robot
from project4b.load_world import load_world
from project4b.Jimmy import Jimmy
from project4b.World import World
from project4b.Helpers import quaternion_from_euler, quaternion_to_euler





class JimmyNode:

    def __init__(self):
        
        # Node creation
        self.node = rclpy.create_node('jimmy_node')

        # Velocity Information
        self.vl = 0
        self.vr = 0
        self.period = 0.1
 
        # Setting up Jimmy and Environment
        self.node.declare_parameter('robot', 'normal.robot.txt')
        self.node.declare_parameter('world', 'brick.world.txt')
        robot = load_disc_robot(self.node.get_parameter('robot').value)
        world = load_world(self.node.get_parameter('world').value)


        self.world = World(world)
        self.jimmy = Jimmy(world['initial_pose'], robot)
        

    

    # -=-=-=-=-=- Initialization -=-=-=-=-=- #
    def start(self):
        self.map_publisher   = self.node.create_publisher(OccupancyGrid, '/map', 10)
        self.scan_publisher  = self.node.create_publisher(LaserScan, '/scan', 10)
        self.vl_subscriber   = self.node.create_subscription(Float64, '/vl', lambda v: self.update_velocity('vl', v), 10)
        self.vr_subscriber   = self.node.create_subscription(Float64, '/vr', lambda v: self.update_velocity('vr', v), 10)
        self.goal_subscriber = self.node.create_subscription(PoseStamped, '/goal_pose', self.teleport, 10)

        self.pose_timer     = self.node.create_timer(self.period, self.pose_controller)
        self.scan_timer     = self.node.create_timer(self.jimmy.laser['rate'], self.scan_controller)
        self.map_timer      = self.node.create_timer(0.5, self.map_controller)
        self.velocity_timer = self.node.create_timer(1.0, self.velocity_timeout)
        self.v_error_timer  = self.node.create_timer(self.jimmy.error_update_rate, self.jimmy.update_wheel_errors)

        self.broadcaster = TransformBroadcaster(self.node, QoSProfile(depth=100))

        self.node.get_logger().warn('Jimmy starting up!')
        rclpy.spin(self.node)

    def done(self):
        self.node.destroy_publisher(self.map_publisher)
        self.node.destroy_publisher(self.scan_publisher)
        self.node.destroy_subscription(self.vl_subscriber)
        self.node.destroy_subscription(self.vr_subscriber)
        self.node.destroy_subscription(self.goal_subscriber)

        self.node.destroy_timer(self.pose_timer)
        self.node.destroy_timer(self.scan_timer)
        self.node.destroy_timer(self.map_timer)
        self.node.destroy_timer(self.velocity_timer)
        self.node.destroy_timer(self.v_error_timer)

        self.node.get_logger().warn('Jimmy is done!')


    # -=-=-=-=-=- Velocity Controllers -=-=-=-=-=- #
    def velocity_timeout(self): 
        self.vl = self.vr = 0

    def update_velocity(self, v_attr, v): 
        # Update the velocity
        setattr(self, v_attr, v.data)

        # Reset the timer
        self.velocity_timer.cancel()
        self.velocity_timer = self.node.create_timer(1.0, self.velocity_timeout)

    def teleport(self, msg): 
        x = msg.pose.position.x
        y = msg.pose.position.y
        o = msg.pose.orientation
        θ = quaternion_to_euler(o.x, o.y, o.z, o.w)

        self.jimmy.pose = (x, y, θ)
        

    # -=-=-=-=-=- Publishers -=-=-=-=-=- #
    def publish_scan(self, ranges): 
        scan = LaserScan()

        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'

        scan.angle_increment = self.jimmy.angle_increment
        scan.angle_min = self.jimmy.laser['angle_min']
        scan.angle_max = self.jimmy.laser['angle_max']
        scan.range_min = self.jimmy.laser['range_min']
        scan.range_max = self.jimmy.laser['range_max']
        scan.ranges = ranges 

        self.scan_publisher.publish(scan)

    def publish_map(self, grid): 

        # Assuming the input 2D array is formatted as array[row][col]
        rows = len(grid)
        cols = len(grid[0])

        # Set the resolution and origin of the map (adjust as needed)
        map_resolution = self.world.resolution  # in meters
        map_origin_x = 0.0  # in meters
        map_origin_y = 0.0  # in meters

        # Create an OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'world'

        occupancy_grid_msg.info.map_load_time = rclpy.clock.Clock().now().to_msg()
        occupancy_grid_msg.info.resolution = map_resolution
        occupancy_grid_msg.info.width = cols
        occupancy_grid_msg.info.height = rows
        occupancy_grid_msg.info.origin.position.x = map_origin_x
        occupancy_grid_msg.info.origin.position.y = map_origin_y
        occupancy_grid_msg.info.origin.position.z = 0.0
        occupancy_grid_msg.info.origin.orientation.x = 0.0
        occupancy_grid_msg.info.origin.orientation.y = 0.0
        occupancy_grid_msg.info.origin.orientation.z = 0.0
        occupancy_grid_msg.info.origin.orientation.w = 1.0

        # Flatten the 2D array and scale the values to the range [0, 100]
        occupancy_grid_msg.data = [cell for row in grid for cell in row]

        self.map_publisher.publish(occupancy_grid_msg)



    # -=-=-=-=-=- Broadcasters -=-=-=-=-=- #
    def broadcast_transform(self, pose): 
        x, y, θ = pose

        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        quaternion = quaternion_from_euler(0, 0, θ)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(t)



    # -=-=-=-=-=- Controllers -=-=-=-=-=- #
    def pose_controller(self): 

        # Handle updating the pose 
        vl, vr      = self.jimmy.skew_velocities(self.vl, self.vr)
        next_pose   = self.jimmy.next_pose(vl, vr, self.period)
        move        = [self.jimmy.pose[:2], next_pose[:2]]

        if self.jimmy.valid_move(self.world.edges, move): 
            self.jimmy.update_pose(next_pose)

        # Broadcast the updated transformation
        self.broadcast_transform(next_pose)

    def scan_controller(self): 
        ranges = self.jimmy.ranges(self.world.edges) 
        ranges = self.jimmy.skew_ranges(ranges)
        self.publish_scan(ranges)

    def map_controller(self): 
        self.publish_map(self.world.map)


    
    







def main(args=None):
    rclpy.init(args=args)

    jimmy_node = JimmyNode()

    try: 
        jimmy_node.start() 
    except KeyboardInterrupt: 
        pass
    finally: 
        jimmy_node.done() 

    
if __name__ == '__main__':
    main()

