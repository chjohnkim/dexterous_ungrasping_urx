#!/usr/bin/env python
import math
import numpy as np

def slerp(v0, v1, t_array):
    # >>> slerp([1,0,0,0],[0,0,0,1],np.arange(0,1,0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0*v1)

    if (dot < 0.0):
        v1 = -v1
        dot = -dot
    
    DOT_THRESHOLD = 0.9995
    if (dot > DOT_THRESHOLD):
        result = v0[np.newaxis,:] + t_array[:,np.newaxis]*(v1 - v0)[np.newaxis,:]
        return (result.T / np.linalg.norm(result, axis=1)).T
    
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0*t_array
    sin_theta = np.sin(theta)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:,np.newaxis] * v0[np.newaxis,:]) + (s1[:,np.newaxis] * v1[np.newaxis,:])

def axis_angle2quaternion(axis, angle):
    '''Convert axis angle representation to quaternion (assumes axis is already normalised).

    Parameters:
        axis (list): 3-D normalised vector of rotation axis (right-hand rule)
        angle (double): Magnitude of tilt angle in degrees
    Returns:
        quaternion (list): quaternion representation in order of qx, qy, qz, qw
    
    '''
    s = math.sin(math.radians(angle)/2)
    qx = axis[0] * s
    qy = axis[1] * s
    qz = axis[2] * s
    qw = math.cos(math.radians(angle)/2)
    return [qx, qy, qz, qw]

def quaternion2axis_angle(q):
    if q[3] > 1:
        print "ERROR: Not unit quaternion!"
        return None
    angle = 2*math.acos(q[3])
    s = math.sqrt(1-q[3]*q[3])
    if s<0.001:
        x = q[0]
        y = q[1]
        z = q[2]
    else:
        x = q[0]/s
        y = q[1]/s
        z = q[2]/s
    return [x, y, z, angle]

def matrix2quaternion(R):
    trace = R[0][0] + R[1][1] + R[2][2] 
    if trace > 0:
        s = 0.5 / math.sqrt(trace+1.0)
        qw = 0.25/s
        qx = (R[1][2] - R[2][1]) * s
        qy = (R[2][0] - R[0][2]) * s
        qz = (R[0][1] - R[1][0]) * s
    else:
        if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
            s = 2.0*math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
            qw = (R[1][2] - R[2][1] ) / s
            qx = 0.25 * s
            qy = (R[1][0] + R[0][1] ) / s
            qz = (R[2][0] + R[0][2] ) / s
        elif R[1][1] > R[2][2]:
            s = 2.0 * math.sqrt( 1.0 + R[1][1] - R[0][0] - R[2][2])
            qw = (R[2][0] - R[0][2] ) / s
            qx = (R[1][0] + R[0][1] ) / s
            qy = 0.25 * s
            qz = (R[2][1] + R[1][2] ) / s
        else: 
            s = 2.0 * math.sqrt( 1.0 + R[2][2] - R[0][0] - R[1][1] )
            qw = (R[0][1] - R[1][0] ) / s
            qx = (R[2][0] + R[0][2] ) / s
            qy = (R[2][1] + R[1][2] ) / s
            qz = 0.25 * s
    return [qx, qy, qz, qw]