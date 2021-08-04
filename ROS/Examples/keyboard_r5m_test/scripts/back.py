#!/usr/bin/env python3

import os, sys, time
import rospy
import serial
import re
import struct
import threading
from blueprintlab_reachsystem_ros_messages.msg import single_float
from pynput.keyboard import Key
from pynput.keyboard import Listener
import numpy as np



class node(object):
    def __init__(self, ACTION_DIM = 4):
        self.node = rospy.init_node("back_r5m")
        self.cmd_position_pub = rospy.Publisher('r5m_0/cmd_position', single_float, queue_size=10)
        self.cmd_position_message = single_float()
        self.sub_pos = rospy.Subscriber('/r5m_0/position', single_float, self.get_position_arm, tcp_nodelay=True, queue_size=10)
        self.ref = np.array([0.5,1.4,0.2,0.3])
        self.position = np.zeros(4)

    def send(self,device, value):
            self.cmd_position_message.device_id = device
            self.cmd_position_message.value = value
            self.cmd_position_pub.publish(self.cmd_position_message)

    def get_position_arm(self, msg):

      
        pose = msg.value
        #print('pose')
        if msg.device_id == 5:
            self.position[0] = pose 
        elif msg.device_id == 4:
            #print('1')
            self.position[1] = pose
        elif msg.device_id == 3:
            #print('2')
            self.position[2] = pose 
        elif msg.device_id == 2:
            #print('3')
            self.position[3] = pose
        

    def loop(self):
        done = False
        r_loop = rospy.Rate(0.05)

        while not rospy.is_shutdown() and done==False:
            
            print('back')
            self.send(5,self.ref[0])
            self.send(4,self.ref[1])
            self.send(3,self.ref[2])
            self.send(2,self.ref[3])
            
            if np.abs(np.linalg.norm(self.ref-self.position))<0.6:
                print('done')
                done = True

            else:
                print('error', np.abs(np.linalg.norm(self.ref-self.position)))


           # r_loop.sleep()


        rospy.signal_shutdown('done')


if __name__ == "__main__":
    
    back = node()
    back.loop()