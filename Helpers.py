import math
from math import sqrt, cos, sin
from numpy import empty


def euclid(p1, p2):
    (x1, y1), (x2, y2) = p1, p2
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def lerp(A, B, t): 
    return A+(B-A)*t

def intersection(s1, s2):
    A, B = s1 
    C, D = s2

    tTop    = (D[0]-C[0])*(A[1]-C[1]) - (D[1]-C[1])*(A[0]-C[0])
    uTop    = (C[1]-A[1])*(A[0]-B[0]) - (C[0]-A[0])*(A[1]-B[1])
    bottom  = (D[1]-C[1])*(B[0]-A[0]) - (D[0]-C[0])*(B[1]-A[1])

    if bottom: 
        t = tTop / bottom
        u = uTop / bottom 

        if (0 <= t <= 1) and (0 <= u <= 1): 
            return (
                lerp(A[0], B[0], t),     
                lerp(A[1], B[1], t),
            )

    return None


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def quaternion_to_euler(x, y, z, w):

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw










