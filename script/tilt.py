#!/usr/bin/env python
import math
import math3d as m3d
import copy
import time

def init_tilt_pose(robot, point, axis, angle, initial_pose, acc=0.1, vel=0.1, wait=True):
    '''3D rotation of robot tcp about a line. 

    Parameters:
        robot: urx.Robot()
        point (list): 3-D coordinate of any point in rotation axis
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tilting 
    Returns:
    
    '''
    T_ = arbitrary_axis_rotation(point, axis, angle)
    pose_target = T_ * initial_pose
    robot.movex(command='movel', pose=pose_target, acc=acc, vel=vel, wait=wait)
    return 0


def tilt_l(robot, point, axis, angle, acc=0.1, vel=0.1, wait=True):
    '''3D rotation of robot tcp about a line. 

    Parameters:
        robot: urx.Robot()
        point (list): 3-D coordinate of any point in rotation axis
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tilting 
    Returns:
    
    '''
    initial_pose = robot.get_pose()
    T_ = arbitrary_axis_rotation(point, axis, angle)
    pose_target = T_ * initial_pose
    robot.movex(command='movel', pose=pose_target, acc=acc, vel=vel, wait=wait)
    return 0


def tilt_c(robot, point, axis, angle, acc=0.1, vel=0.1, radius=0.005, wait=True):
    '''3D rotation of robot tcp about a line. 

    Parameters:
        robot: urx.Robot()
        point (list): 3-D coordinate of any point in rotation axis
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tilting 
    Returns:
    
    '''
    initial_pose = robot.get_pose()
    waypoints = []
    for theta in range(1,angle+1):
        T_ = arbitrary_axis_rotation(point, axis, theta)
        pose_via = T_ * initial_pose
        waypoints.append(copy.deepcopy(pose_via))  
    robot.movexs(command='movel', pose_list=waypoints, acc=acc, vel=vel, radius=radius, wait=wait)
    return 0

def arbitrary_axis_rotation(point, axis, theta):
    '''Returns SE(3) transformation matrix for rotating about an arbitrary axis in 3D.
    Refer to: https://sites.google.com/site/glennmurray/Home/rotation-matrices-and-formulas/rotation-about-an-arbitrary-axis-in-3-dimensions

    Parameters:
        point (list): 3-D coordinate of any point in rotation axis
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle magnitude to rotate in degrees 
    Returns:
        T_ (m3d.Trnasform): SE(3) transformation mtarix 
    '''
    a, b, c = point
    uUn, vUn, wUn = axis
    l = math.sqrt(uUn*uUn + vUn*vUn + wUn*wUn)
    
    # In this instance we normalize the direction vector.
    u = uUn/l
    v = vUn/l
    w = wUn/l
    
    # Set some intermediate values.
    u2 = u*u
    v2 = v*v
    w2 = w*w
    cosT = math.cos(math.radians(theta))
    oneMinusCosT = 1-cosT
    sinT = math.sin(math.radians(theta))
    
    # Build the matrix entries element by element.
    m11 = u2 + (v2 + w2) * cosT
    m12 = u*v * oneMinusCosT - w*sinT
    m13 = u*w * oneMinusCosT + v*sinT
    m14 = (a*(v2 + w2) - u*(b*v + c*w))*oneMinusCosT + (b*w - c*v)*sinT
    m21 = u*v * oneMinusCosT + w*sinT
    m22 = v2 + (u2 + w2) * cosT
    m23 = v*w * oneMinusCosT - u*sinT
    m24 = (b*(u2 + w2) - v*(a*u + c*w))*oneMinusCosT + (c*u - a*w)*sinT
    m31 = u*w * oneMinusCosT - v*sinT
    m32 = v*w * oneMinusCosT + u*sinT
    m33 = w2 + (u2 + v2) * cosT
    m34 = (c*(u2 + v2) - w*(a*u + b*v))*oneMinusCosT + (a*v - b*u)*sinT

    # Populate transformation matrix
    T_ = m3d.Transform()
    T_.orient[0][0] = m11
    T_.orient[1][0] = m12
    T_.orient[2][0] = m13
    T_.orient[0][1] = m21
    T_.orient[1][1] = m22
    T_.orient[2][1] = m23
    T_.orient[0][2] = m31
    T_.orient[1][2] = m32
    T_.orient[2][2] = m33
    T_.pos[0] = m14
    T_.pos[1] = m24
    T_.pos[2] = m34
    
    return T_
 