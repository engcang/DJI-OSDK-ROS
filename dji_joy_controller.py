#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 16 10:10:09 2022
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin
import numpy as np

import rospy
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from dji_osdk_ros.srv import FlightTaskControl, ObtainControlAuthority, SetLocalPosRef

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.position_pub = rospy.Publisher('/dji_osdk_ros/set_local_pose', PoseStamped, queue_size=10)
        self.pos_sub = rospy.Subscriber('/dji_osdk_ros/local_odom', Odometry, self.pose_callback)
        self.status_sub = rospy.Subscriber('/dji_osdk_ros/flight_status', UInt8, self.status_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_srv = rospy.ServiceProxy('/flight_task_control', FlightTaskControl)
        self.offboard_srv = rospy.ServiceProxy('/obtain_release_control_authority', ObtainControlAuthority)
        self.local_ref_set_srv = rospy.ServiceProxy('/set_local_pos_reference', SetLocalPosRef)

        self.rate = rospy.Rate(20)
        self.hold = 0 #to stop sending input shortly
        self.mode = 0 #0: position, 1: velocity, 2: attitude, 3: pqrt
        self.joy_check=0
        self.mav_check=0
        self.stat_check=0
        self.max_vel_x = 5
        self.max_vel_y = 5
        self.max_vel_z = 3
        self.yaw_rate = 1.5

    def status_callback(self, msg):
        self.stat = msg.data
        self.stat_check=1
    def pose_callback(self, msg):
        self.truth=msg.pose.pose.position
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.mav_check=1

    def joy_callback(self, msg):
        self.joy = msg
        if len(msg.axes)>0 or len(msg.buttons)>0:
            if self.stat_check==1:
                if msg.buttons[2]==1:
                    if self.stat==0: #stopped
                        self.cmd_srv(task=7) #start motor
                    elif self.stat==1: #armed, ground (cf. 2: flying in air)
                        self.cmd_srv(task=8) #stop motor
                if msg.buttons[3]==1:
                    self.offboard_srv(enable_obtain=True) #obtain control authority
                if msg.buttons[5]==1:
                        self.cmd_srv(task=4) #take off
                if msg.buttons[4]==1:
                        self.cmd_srv(task=3) #homing and landing
                if msg.buttons[7]==1:
                        self.local_ref_set_srv() #Local Reference GPS position set
            if msg.buttons[0]==1:
                self.hold = self.hold+1
            if msg.buttons[1]==1:
                self.mode = self.mode+1
                if self.mode==4:
                    self.mode = 0
            self.joy_check=1

    def input(self):
        if self.hold%2==0:
            if self.mode==0:
                pose_input=PoseStamped()

                pose_input.pose.position.x = self.truth.x + ( self.joy.axes[4]*self.max_vel_x)*cos(self.yaw) - ( self.joy.axes[3]*self.max_vel_y)*sin(self.yaw)
                pose_input.pose.position.y = self.truth.y + ( self.joy.axes[3]*self.max_vel_y)*cos(self.yaw) + ( self.joy.axes[4]*self.max_vel_x)*sin(self.yaw)
                pose_input.pose.position.z = self.truth.z + ( self.joy.axes[1]*self.max_vel_z)
                yaw_input = rpy_saturation(self.yaw + self.yaw_rate*(self.joy.axes[0]))
                qq = quaternion_from_euler(0,0,yaw_input)
                pose_input.pose.orientation.x = qq[0]
                pose_input.pose.orientation.y = qq[1]
                pose_input.pose.orientation.z = qq[2]
                pose_input.pose.orientation.w = qq[3]

                pose_input.header.stamp = rospy.Time.now()
                self.position_pub.publish(pose_input)

            elif self.mode==1:

            elif self.mode==2:

            elif self.mode==3:


            print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(self.truth.x, self.truth.y, self.truth.z))
            print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(self.roll/np.pi*180, self.pitch/np.pi*180, self.yaw/np.pi*180)) #radian : +-pi
        else:
            print("Hold now, press Button[0] to control again")
            print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(self.truth.x, self.truth.y, self.truth.z))
            print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(self.roll/np.pi*180, self.pitch/np.pi*180, self.yaw/np.pi*180)) #radian : +-pi

##############################################################################################

uav_ctr = robot()
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if uav_ctr.joy_check==1 and uav_ctr.mav_check==1 and uav_ctr.stat_check==1:
                uav_ctr.input()
            else: 
                pass
            uav_ctr.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
