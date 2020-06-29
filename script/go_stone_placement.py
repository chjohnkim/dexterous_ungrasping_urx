#!/usr/bin/env python
import sys
import time
import yaml
import urx
import numpy as np
import math3d as m3d
import rospy
import tilt
import regrasp

import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
rospy.init_node('dexterous_ungrasping', anonymous=True)  
action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()

if __name__ == '__main__':
    with open("/home/john/Desktop/dexterous_ungrasping_urx/config/du_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Read parameters from config file
    tcp_acc = config['tcp_acc']
    tcp_vel = config['tcp_vel']
    theta_0 = config['theta_0']
    delta_0 = config['delta_0']
    psi_regrasp = config['psi_regrasp']
    theta_tilt = config['theta_tilt']
    tuck_angle = config['tuck']
    axis =  config['axis']
    object_thickness = config['object_thickness']
    object_length = config['object_length']
    tcp2fingertip = config['tcp2fingertip']

    # Initiate Robot
    robot = urx.Robot("192.168.1.102")
    robot.set_tcp((0,0,0,0,0,0))
    Robotiq.goto(robotiq_client, pos=object_thickness+0.004, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
    print "Robot Initated Successfully!"
    
    # Go to home_pose
    home_pose = m3d.Transform()
    home_pose.orient[0][0] = 0
    home_pose.orient[1][0] = 1
    home_pose.orient[0][1] = 1
    home_pose.orient[1][1] = 0
    home_pose.orient[2][2] = -1
    home_pose.pos = [-0.32515, 0.33697, 0.400]
    robot.movex(command='movel', pose=home_pose, acc=tcp_acc, vel=tcp_vel)
    
    # Tilt
    current_pose = robot.get_pose()
    tilt_point_e = m3d.Vector(0, 0, tcp2fingertip) 
    tilt_point_w = current_pose*tilt_point_e
    tilt.tilt_l(robot, point=tilt_point_w, axis=axis, angle=90-theta_0, acc=tcp_acc, vel=tcp_vel)
    #tilt.tilt(robot, point=tilt_point_w, axis=axis, angle=20, acc=tcp_acc, vel=tcp_vel)
     
    robot.movex(command='movel', pose=(0,0,-0.073, 0, 0, 0), acc=tcp_acc, vel=tcp_vel, wait=True, relative=True, threshold=None)
       
    # Regrasp
    width = regrasp.regrasp(robot, axis=np.multiply(axis, -1), angle=psi_regrasp, psi_init=0, acc=tcp_acc, vel=tcp_vel)
    
    # Tilt
    tilt_point_w[2]=tilt_point_w[2]-0.073
    tilt.tilt_c(robot, point=tilt_point_w, axis=axis, angle=theta_tilt, acc=tcp_acc, vel=tcp_vel)
    
    # Second regrasp
    regrasp.second_regrasp(robot, axis=np.multiply(axis, -1), angle=tuck_angle, width_init=width, theta_init=theta_0+psi_regrasp-theta_tilt, psi_init=psi_regrasp, acc=tcp_acc, vel=tcp_vel)
    
    robot.movex(command='movel', pose=(0,0, 0.1, 0, 0, 0), acc=tcp_acc, vel=tcp_vel, wait=True, relative=True, threshold=None)
    
    robot.close()
    sys.exit()