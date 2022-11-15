import math
from math import (
    asin, pi, atan2, cos 
)
 
def euler_from_quaternion(x, y, z, w):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q

def rotation_matrix_to_euler(rotation_matrix):
    """
    R = [ 
       [R11 , R12 , R13], 
       [R21 , R22 , R23],
       [R31 , R32 , R33]  
    ]
    """
    R11 = rotation_matrix[0][0]
    R12 = rotation_matrix[0][1]
    R13 = rotation_matrix[0][2]
    R21 = rotation_matrix[1][0]
    R22 = rotation_matrix[1][1]
    R23 = rotation_matrix[1][2]
    R31 = rotation_matrix[2][0]
    R32 = rotation_matrix[2][1]
    R33 = rotation_matrix[2][2]

    if R31 != 1 and R31 != -1: 
        pitch_1 = -1*asin(R31)
        pitch_2 = pi - pitch_1 
        roll_1 = atan2( R32 / cos(pitch_1) , R33 /cos(pitch_1) ) 
        roll_2 = atan2( R32 / cos(pitch_2) , R33 /cos(pitch_2) ) 
        yaw_1 = atan2( R21 / cos(pitch_1) , R11 / cos(pitch_1) )
        yaw_2 = atan2( R21 / cos(pitch_2) , R11 / cos(pitch_2) ) 

        # IMPORTANT NOTE here, there is more than one solution but we choose the first for this case for simplicity !
        # You can insert your own domain logic here on how to handle both solutions appropriately (see the reference publication link for more info). 
        pitch = pitch_1 
        roll = roll_1
        yaw = yaw_1 
    else: 
        yaw = 0 # anything (we default this to zero)
        if R31 == -1: 
            pitch = pi/2 
            roll = yaw + atan2(R12,R13) 
        else: 
            pitch = -pi/2 
            roll = -1*yaw + atan2(-1*R12,-1*R13) 

    # convert from radians to degrees
    roll = roll*180/pi 
    pitch = pitch*180/pi
    yaw = yaw*180/pi 

    rxyz_deg = [roll , pitch , yaw] 

    return rxyz_deg