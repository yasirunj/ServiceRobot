#!/usr/bin/env python3

import rospy
import numpy as np
from simple_pid import PID

from geometry_msgs.msg import Twist

pid_Left = PID(1, 0.1, 0.05, setpoint=1)
pid_Right = PID(1, 0.1, 0.05, setpoint=1)

global msg

def velCmdCallBack(velCmd):
    global demandx, demandz

    demandx = velCmd.linear.x
    demandz = velCmd.angular.z

def velCallBack(vel):
    global supplyx, supplyz

    supplyx = vel.linear.x
    supplyz = vel.angular.z

def controller():
    pub = rospy.Publisher('motor_pub', Vector3, queue_size=10)

    demandLeft = demandx - (demandz*0.32)
    demandRight = demandx + (demandz*0.32)

    supplyLeft = supplyx - (supplyz*0.32)
    supplyRight = supplyx + (supplyz*0.32)

    pid_Left.setpoint = demandLeft
    pid_Right.setpoint = demandRight 

    controlled_Left = pid_Left(supplyLeft)   
    controlled_Right = pid_Right(supplyRight)  

    msg.x = controlled_Left
    msg.y = controlled_Right

    pub.publish()

def listener():
    rospy.init_node('diff_motor_controller', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, velCmdCallBack)
    rospy.Subscriber("vel_pub", Twist, velCallBack)

    rospy.spin()

if __name__ == '__main__':
    listener()