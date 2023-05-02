# -----------------------------------------
# This code is to control UR5e using the Python MoveIt user interfaces
# -----------------------------------------

#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from math import pi
import tf
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

from twisted.internet import reactor
from pymodbus.client.asynchronous import schedulers
from pymodbus.client.asynchronous.serial import AsyncModbusSerialClient
from pymodbus.client.asynchronous.twisted import ModbusClientProtocol

from utils import MoveGroupInteface

# -----------------------------------------
# initialize
# -----------------------------------------
print('Control UR5e')
print('-'*30)
demo = MoveGroupInteface()
print('-'*30)

# -----------------------------------------
# ungrasp
# -----------------------------------------

# -----------------------------------------
# get joint status (pose for avoidance) 
# -----------------------------------------
demo.plan_joint_goal(0, -1.8, 1.4, -1.18, -1.56, 0)
print('pose for avoidance is done!')
print('-'*30)
demo.ungrasp()
# -----------------------------------------
# get joint status (pose for observation) 
# -----------------------------------------
# demo.plan_joint_goal(0, -0.9, 0.5, -1.57, -1.57, 0)
print('pose for observation is done!')
print('-'*30)

# -----------------------------------------
# get object position in real time
# -----------------------------------------

z_init = 0
x_init = 0.66
y_init = -0.33
# z=0.140453006775
print('target object position: x={}, y={}, z={}'.format(x_init, y_init, z_init))

# -----------------------------------------
# get joint status (pose for grasping) 
# -----------------------------------------
demo.plan_joint_goal(0, -0.9, 0.5, -1.2, -1.57, 0)
print('pose for grasping is done!')
print('-'*30)

# -----------------------------------------
# execute trajectories
# -----------------------------------------
print('execute computed trajectory')
# raw_input()
x, y, z = demo.point1_astra(x_init, y_init, z_init)
print('x:{} y:{} z:{}'.format(x, y, z))
cartesian_plan, fraction = demo.plan_cartesian_path(x, y, z)
demo.execute_plan(cartesian_plan)
print('-'*30)

x, y, z = demo.point2(x, y, z)
print('x:{} y:{} z:{}'.format(x, y, z))
cartesian_plan, fraction = demo.plan_cartesian_path(x, y, z)
demo.execute_plan(cartesian_plan)
print('-'*30)

x, y, z = demo.point3(x, y, z)
print('x:{} y:{} z:{}'.format(x, y, z))
cartesian_plan, fraction = demo.plan_cartesian_path(x, y, z)
demo.execute_plan(cartesian_plan)
print('-'*30)

x, y, z = demo.point4(x, y, z)
print('x:{} y:{} z:{}'.format(x, y, z))
cartesian_plan, fraction = demo.plan_cartesian_path(x, y, z)
demo.execute_plan(cartesian_plan)
print('-'*30)

x, y, z = demo.point5(x, y, z)
print('x:{} y:{} z:{}'.format(x, y, z))
cartesian_plan, fraction = demo.plan_cartesian_path(x, y, z)
demo.execute_plan(cartesian_plan)
print('-'*30)

# demo.rotate_hand()

x, y, z = demo.point6(x, y, z)
print('x:{} y:{} z:{}'.format(x, y, z))
cartesian_plan, fraction = demo.plan_cartesian_path(x, y, z)
demo.execute_plan(cartesian_plan)
print('-'*30)


# -----------------------------------------
# grasp
# -----------------------------------------

print('grasp is done!')
print('-'*30)

# -----------------------------------------
# get joint status (pose for observation) 
# -----------------------------------------
demo.ungrasp()
print('ungrasp is done!')
print('-'*30)
demo.plan_joint_goal(0, -0.9, 0.5, -1.57, -1.57, 0)
print('pose for observation is done!')
