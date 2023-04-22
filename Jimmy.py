
from math import cos, sin
from project4b.Helpers import euclid, intersection
from random import random, gauss




class Jimmy: 
    def __init__(self, pose, robot):
        self.pose = tuple(pose)
        self.body = robot['body']
        self.laser = robot['laser']
        self.wheels = robot['wheels'] 

        # Body values
        self.radius = robot['body']['radius']

        # Laser values
        self.angle_increment = (robot['laser']['angle_max'] - robot['laser']['angle_min']) / robot['laser']['count']
        self.angle_min = pose[2] + robot['laser']['angle_min']
        self.angle_max = pose[2] + robot['laser']['angle_max']
        self.laser_fail_probability = robot['laser']['fail_probability']
        self.laser_error_variance = robot['laser']['error_variance']

        # Wheel values
        self.error_variance_left = robot['wheels']['error_variance_left']
        self.error_variance_right = robot['wheels']['error_variance_right']
        self.error_update_rate = robot['wheels']['error_update_rate']
        self.vl_error = 1.00
        self.vr_error = 1.00

        print("Jimmy is ready!")


    def lidar_pose(self): 
        x, y, θ = self.pose 
        r = self.radius/2

        return (
            x + r*cos(θ), 
            y + r*sin(θ), 
            θ
        )

    def update_pose(self, pose): 
        self.pose = pose
        self.generate_rays()

    def generate_rays(self): 
        rays = []
        x, y, θ = self.lidar_pose()
        r1 = self.laser['range_min']
        r2 = self.laser['range_max']
        angle_min = θ + self.laser['angle_min']

        for i in range(self.laser['count'] + 1):
            theta = angle_min + self.angle_increment*i
            x1 = r1 * cos(theta) + x
            y1 = r1 * sin(theta) + y
            x2 = r2 * cos(theta) + x
            y2 = r2 * sin(theta) + y

            ray = [(x1, y1), (x2, y2)]
            rays.append(ray)
        
        return rays
    
    def ranges(self, edges): 

        ranges = [] 
        center = self.lidar_pose()[:2]

        for ray in self.generate_rays(): 

            # Calculate the intersections
            intersections = []
            for edge in edges: 
                point = intersection(ray, edge)
                if point is not None: 
                    intersections.append(point)


            # Calculate the nearest point
            nearest_dist = float('inf')

            for point in intersections: 
                distance = euclid(point, center)
                if distance < nearest_dist: 
                    nearest_dist = distance

            ranges.append(nearest_dist)
        


        return ranges

    def next_pose(self, vl, vr, t): 

        l = self.wheels['distance']
        x, y, θ = self.pose

        if vr == vl:    # Moving in straight line
            if vl == 0: 
                return (x, y, θ)
            
            d = -1 if vl < 0 else 1
            xp = x + d*cos(θ)*t
            yp = y + d*sin(θ)*t
            θp = θ
            return (xp, yp, θp)


        w = (vr - vl) / l
        R = (l/2) * ((vr+vl) / (vr-vl))

        cx = x - R*sin(θ)
        cy = y + R*cos(θ)

        a, b    = cos(w*t), sin(w*t)
        dx, dy  = x-cx, y-cy

        xp = a*dx - b*dy + cx 
        yp = b*dx + a*dy + cy
        θp = θ + w*t 

        return (xp, yp, θp)

    def valid_move(self, edges, move): 

        r = self.radius
        c1, c2 = move
        a, d = (c2[0]-r, c2[1]), (c2[0], c2[1]+r)
        b, c = (c2[0]+r, c2[1]), (c2[0], c2[1]-r)

        for edge in edges:
            p1, p2 = edge

            if ((a[0] <= p1[0] <= c1[0] or c1[0] <= p1[0] <= b[0]) and p1[1] <= c[1] <= d[1] <= p2[1]) or \
               ((c[1] <= p1[1] <= c1[1] or c1[1] <= p1[1] <= d[1]) and p1[0] <= a[0] <= b[0] <= p2[0]): 
                return False

            if euclid(c2, p1) <= r or euclid(c2, p2) <= r:
                return False

        return True


    def skew_ranges(self, ranges): 

        skewed_ranges = []
        for r in ranges: 
            
            if random() < self.laser_fail_probability:   # Measurement failed
                skewed_ranges.append(float('nan'))
                continue 
            
            skewed_range = r + gauss(0, self.laser_error_variance)
            skewed_ranges.append(skewed_range)

        return skewed_ranges

    def skew_velocities(self, vl, vr): 
        vl = vl * self.vl_error if vl != 0 else 0
        vr = vr * self.vr_error if vr != 0 else 0
        return vl, vr
    
    def update_wheel_errors(self):
        self.vl_error = gauss(1, self.error_variance_left)
        self.vr_error = gauss(1, self.error_variance_right)

   

        




