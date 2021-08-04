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



class keyboard_cpt(object):
    def __init__(self):
        self.velocity = np.zeros(5)
        self.delta_vel = 0.1
        self.cmd_velocity_pub = rospy.Publisher('r5m_0/cmd_velocity', single_float, queue_size=10)
        self.cmd_velocity_message = single_float()

    def positive_key(self,dev_id):
        self.velocity[dev_id-1] = np.clip(self.velocity[dev_id-1] + self.delta_vel,-0.6, 0.6)
        self.cmd_velocity_message.device_id = dev_id
        self.cmd_velocity_message.value = self.velocity[dev_id-1]
        self.cmd_velocity_pub.publish(self.cmd_velocity_message)
        return 

    def negative_key(self,dev_id):
        self.velocity[dev_id-1] = np.clip(self.velocity[dev_id-1] - self.delta_vel,-0.6, 0.6)
        self.cmd_velocity_message.device_id = dev_id
        self.cmd_velocity_message.value = self.velocity[dev_id-1]
        self.cmd_velocity_pub.publish(self.cmd_velocity_message)
        return 
    def key_released(self, dev_id):
        self.velocity[dev_id-1] = 0.
        self.cmd_velocity_message.device_id = dev_id
        self.cmd_velocity_message.value = self.velocity[dev_id-1]
        self.cmd_velocity_pub.publish(self.cmd_velocity_message)
        return

    def keyboard_press(self,key):
        try:
            key_pressed = key.char
        except:
            return
        if key.char == 'w':  
            dev_id = 3 
            self.positive_key(dev_id)
            print('w pressed')
        elif key.char == 's':
            dev_id = 3 
            self.negative_key(dev_id)
            print('s pressed')
        
        elif key.char == 'a':
            dev_id = 5 
            self.positive_key(dev_id)
            print('a pressed')
        elif key.char == 'd':  
            dev_id = 5 
            self.negative_key(dev_id)
            print('d pressed')        
        elif key.char == 'i': 
            dev_id = 1 
            self.positive_key(dev_id)
            print('i pressed')
        elif key.char == 'k':
            dev_id = 1 
            self.negative_key(dev_id)
            print('k pressed')
        elif key.char == 'j':
            dev_id = 2 
            self.positive_key(dev_id)                   
            print('j pressed')
        elif key.char == 'l':        
            dev_id = 2 
            self.negative_key(dev_id)
            print('l pressed') 
        elif key.char == 'q': 
            dev_id = 4 
            self.positive_key(dev_id)
            print('q pressed')
        elif key.char == 'e':   
            dev_id = 4 
            self.negative_key(dev_id)      
            print('e pressed')     
                    

    def keyboard_release(self,key):    
        if key == Key.esc:
            return False
        try:
            key_released = key.char
        except:
            return
        if key.char == 'w':    
            dev_id = 3  
            self.key_released(dev_id)
            print('w released')
        elif key.char == 's':        
            dev_id = 3  
            self.key_released(dev_id)
            print('s released')
        
        elif key.char == 'a':        
            dev_id = 5  
            self.key_released(dev_id)
            print('a released')
        elif key.char == 'd':        
            dev_id = 5  
            self.key_released(dev_id)
            print('d released') 
        elif key.char == 'i':        
            dev_id = 1  
            self.key_released(dev_id)
            print('a released')
        elif key.char == 'k':        
            dev_id = 1
            self.key_released(dev_id)
            print('d released') 
        elif key.char == 'j':        
            dev_id = 2  
            self.key_released(dev_id)
            print('a released')
        elif key.char == 'l':        
            dev_id = 2
            self.key_released(dev_id)
            print('d released')             
        elif key.char == 'q':        
            dev_id = 4  
            self.key_released(dev_id)
            print('a released')
        elif key.char == 'e':        
            dev_id = 4  
            self.key_released(dev_id)
            print('d released')   
         

         


    def keyboard_r5m_main(self):

        if not rospy.is_shutdown():
            with Listener(
                        on_press=self.keyboard_press,
                        on_release=self.keyboard_release
                        ) as listener:
                listener.join()

        


        
        #cmd_velocity_message.device_id = 3
        #cmd_velocity_message.value = 0.5
        #cmd_velocity_pub.publish(cmd_velocity_message)
        #time.sleep(3.0)
        #cmd_velocity_message.device_id = 3
        #cmd_velocity_message.value = -0.5
        #cmd_velocity_pub.publish(cmd_velocity_message)
        #time.sleep(3.0)




if __name__ == "__main__":
    rospy.init_node("keyboard_r5m")
    key = keyboard_cpt()
    key.keyboard_r5m_main()
