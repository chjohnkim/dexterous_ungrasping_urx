#!/usr/bin/env python
import math
import rospy
import copy
import math3d as m3d
import numpy as np
import tf
import yaml
import helper

import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


def regrasp(robot, axis, angle, psi_init=0, acc=0.1, vel=0.1, wait=True):
    with open("/home/john/Desktop/dexterous_ungrasping_urx/config/du_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    
    tcp2fingertip = config['tcp2fingertip']
    init_pose = robot.get_pose()
    contact_A_e = m3d.Vector(config['object_thickness']/2, 0, tcp2fingertip) #TODO: depends on axis direction and gripper orientation. Refer to gripper axis for now and adjust accordingly.
    contact_A_w = init_pose*contact_A_e 
    
    # Interpolate orientation poses via quaternion slerp
    q_initial = helper.matrix2quaternion(init_pose.orient)
    q = helper.axis_angle2quaternion(axis, angle)
    q_target = tf.transformations.quaternion_multiply(q, q_initial )    
    q_waypoints = helper.slerp(q_initial, q_target, np.arange(1.0/angle , 1.0+1.0/angle, 1.0/angle)) 
    
    theta_0 = config['theta_0']    
    waypoints = []
    pose_via = init_pose.copy()
    for psi in range(psi_init+1, angle+1):
        # Calculate width
        a = config['delta_0'] * math.cos(math.radians(psi))
        b = config['delta_0'] * math.sin(math.radians(psi))
        c = config['object_thickness'] * math.cos(math.radians(psi))
        d = config['object_thickness'] * math.sin(math.radians(psi))
        opposite = a - d
        width = b + c

        # Calculate position 
        if theta_0 + psi <= 90:
            hori =  math.fabs(tcp2fingertip*math.cos(math.radians(theta_0 + psi))) + math.fabs((width/2.0)*math.sin(math.radians(theta_0+psi)))
            verti =  math.fabs(tcp2fingertip*math.sin(math.radians(theta_0 + psi))) - math.fabs((width/2.0)*math.cos(math.radians(theta_0+psi)))
        else:
            hori = -math.fabs(tcp2fingertip*math.sin(math.radians(theta_0 + psi-90))) + math.fabs((width/2.0)*math.cos(math.radians(theta_0+psi-90)))
            verti = math.fabs(tcp2fingertip*math.cos(math.radians(theta_0 + psi-90))) + math.fabs((width/2.0)*math.sin(math.radians(theta_0+psi-90)))
        
        if axis[0] > 0:
            pose_via.pos[1] = contact_A_w[1] + hori
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 1"
        elif axis[0] < 0:
            pose_via.pos[1] = contact_A_w[1] - hori 
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 2"
        elif axis[1] > 0:
            pose_via.pos[0] = contact_A_w[0] - hori
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 3"
        elif axis[1] < 0:
            pose_via.pos[0] = contact_A_w[0] + hori
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 4"
        
        pose_via.orient =  tf.transformations.quaternion_matrix(q_waypoints[psi-psi_init-1])[:3,:3]
        waypoints.append(copy.deepcopy(pose_via))
    #print init_pose
    #print waypoints
    
    robot.movexs(command='movel', pose_list=waypoints, acc=acc, vel=vel, radius=0.005, wait=False)

    psi = 0
    while psi < angle:
        pose_current = robot.get_pose()
        q_current = helper.matrix2quaternion(pose_current.orient)
        psi = 2*math.degrees(math.acos(np.dot(q_current, q_initial)))
        if psi > 100:
            psi = -(psi-360)
        a = config['delta_0'] * math.cos(math.radians(psi))
        b = config['delta_0'] * math.sin(math.radians(psi))
        c = config['object_thickness'] * math.cos(math.radians(psi))
        d = config['object_thickness'] * math.sin(math.radians(psi))
        opposite = a - d
        width = b + c + 0.004
        Robotiq.goto(robotiq_client, pos=width, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        psi = round(psi, 2)
        rospy.sleep(0.5)
    return width
    

def second_regrasp(robot, axis, angle, width_init, theta_init, psi_init, acc=0.1, vel=0.1, wait=True):
    with open("/home/john/Desktop/dexterous_ungrasping_urx/config/du_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    
    tcp2fingertip = config['tcp2fingertip']
    init_pose = robot.get_pose()
    contact_A_e = m3d.Vector(width_init/2, 0, tcp2fingertip) #TODO: depends on axis direction and gripper orientation. Refer to gripper axis for now and adjust accordingly.
    contact_A_w = init_pose*contact_A_e 
    
    # Interpolate orientation poses via quaternion slerp
    q_initial = helper.matrix2quaternion(init_pose.orient)
    q = helper.axis_angle2quaternion(axis, angle)
    q_target = tf.transformations.quaternion_multiply(q, q_initial )    
    q_waypoints = helper.slerp(q_initial, q_target, np.arange(1.0/angle , 1.0+1.0/angle, 1.0/angle)) 
    
    theta_0 = theta_init    
    waypoints = []
    pose_via = init_pose.copy()
    for psi in range(1, angle+1):
        # Calculate width
        a = config['delta_0'] * math.cos(math.radians(psi+psi_init))
        b = config['delta_0'] * math.sin(math.radians(psi+psi_init))
        c = config['object_thickness'] * math.cos(math.radians(psi+psi_init))
        d = config['object_thickness'] * math.sin(math.radians(psi+psi_init))
        opposite = a - d
        width = b + c

        # Calculate position 
        if theta_0 + psi <= 90:
            hori =  math.fabs(tcp2fingertip*math.cos(math.radians(theta_0 + psi))) + math.fabs((width/2.0)*math.sin(math.radians(theta_0+psi)))
            verti =  math.fabs(tcp2fingertip*math.sin(math.radians(theta_0 + psi))) - math.fabs((width/2.0)*math.cos(math.radians(theta_0+psi)))
        else:
            hori = -math.fabs(tcp2fingertip*math.sin(math.radians(theta_0 + psi-90))) + math.fabs((width/2.0)*math.cos(math.radians(theta_0+psi-90)))
            verti = math.fabs(tcp2fingertip*math.cos(math.radians(theta_0 + psi-90))) + math.fabs((width/2.0)*math.sin(math.radians(theta_0+psi-90)))
        
        if axis[0] > 0:
            pose_via.pos[1] = contact_A_w[1] + hori
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 1"
        elif axis[0] < 0:
            pose_via.pos[1] = contact_A_w[1] - hori 
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 2"
        elif axis[1] > 0:
            pose_via.pos[0] = contact_A_w[0] - hori
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 3"
        elif axis[1] < 0:
            pose_via.pos[0] = contact_A_w[0] + hori
            pose_via.pos[2] = contact_A_w[2] + verti
            #print "CASE 4"
        
        pose_via.orient =  tf.transformations.quaternion_matrix(q_waypoints[psi-1])[:3,:3]
        waypoints.append(copy.deepcopy(pose_via))
    #print init_pose
    #print waypoints
    
    robot.movexs(command='movel', pose_list=waypoints, acc=acc, vel=vel, radius=0.005, wait=False)

    psi = 0
    while psi < angle:
        pose_current = robot.get_pose()
        q_current = helper.matrix2quaternion(pose_current.orient)
        psi = 2*math.degrees(math.acos(np.dot(q_current, q_initial)))
        if psi > 100:
            psi = -(psi-360)
        a = config['delta_0'] * math.cos(math.radians(psi+psi_init))
        b = config['delta_0'] * math.sin(math.radians(psi+psi_init))
        c = config['object_thickness'] * math.cos(math.radians(psi+psi_init))
        d = config['object_thickness'] * math.sin(math.radians(psi+psi_init))
        opposite = a - d
        width = b + c + 0.005
        Robotiq.goto(robotiq_client, pos=width, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        psi = round(psi, 2)
        rospy.sleep(0.5)
    return 0
