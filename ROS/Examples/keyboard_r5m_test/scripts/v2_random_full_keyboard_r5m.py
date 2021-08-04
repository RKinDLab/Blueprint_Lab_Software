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

cmd_velocity_pub = rospy.Publisher('r5m_0/cmd_velocity', single_float, queue_size=10)
cmd_velocity_message = single_float()
class keys(object):
    def __init__(self):
        self.count = 0
        self.rand_values = 0.4*np.random.rand(6)

    def keyboard_press(self,key):

        self.count += 1
        if self.count > 100:
            self.count = 0
            self.rand_values = 0.4*np.random.rand(6)

        try:
            key_pressed = key.char
        except:
            return
        if key.char == 'w':        
            cmd_velocity_message.device_id = 3
            cmd_velocity_message.value = self.rand_values[3]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('w pressed')
        elif key.char == 's':        
            cmd_velocity_message.device_id = 3
            cmd_velocity_message.value = -self.rand_values[3]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('s pressed')
        elif key.char == 'a':        
            cmd_velocity_message.device_id = 5
            cmd_velocity_message.value = self.rand_values[5]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('a pressed')
        elif key.char == 'd':        
            cmd_velocity_message.device_id = 5
            cmd_velocity_message.value = -self.rand_values[5]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('d pressed')        
        elif key.char == 'i':        
            cmd_velocity_message.device_id = 1
            cmd_velocity_message.value = self.rand_values[1]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('a pressed')
        elif key.char == 'k':        
            cmd_velocity_message.device_id = 1
            cmd_velocity_message.value = -self.rand_values[1]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('d pressed')
        elif key.char == 'j':        
            cmd_velocity_message.device_id = 2
            cmd_velocity_message.value = self.rand_values[2]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('a pressed')
        elif key.char == 'l':        
            cmd_velocity_message.device_id = 2
            cmd_velocity_message.value = -self.rand_values[2]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('d pressed') 
        elif key.char == 'q':        
            cmd_velocity_message.device_id = 4
            cmd_velocity_message.value = self.rand_values[4]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('a pressed')
        elif key.char == 'e':        
            cmd_velocity_message.device_id = 4
            cmd_velocity_message.value = -self.rand_values[4]
            cmd_velocity_pub.publish(cmd_velocity_message)
            print('d pressed')                 

def keyboard_release(key):    
    if key == Key.esc:
        return False
    try:
        key_released = key.char
    except:
        return
    if key.char == 'w':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('s pressed')
    elif key.char == 'a':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'd':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed') 
    elif key.char == 'i':        
        cmd_velocity_message.device_id = 1
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'k':        
        cmd_velocity_message.device_id = 1
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed') 
    elif key.char == 'j':        
        cmd_velocity_message.device_id = 2
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'l':        
        cmd_velocity_message.device_id = 2
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed')             
    elif key.char == 'q':        
        cmd_velocity_message.device_id = 4
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'e':        
        cmd_velocity_message.device_id = 4
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed')    

     


def keyboard_r5m_main():

    k = keys()
    if not rospy.is_shutdown():
        with Listener(
                    on_press=k.keyboard_press,
                    on_release=keyboard_release
                    ) as listener:
            listener.join()
        print('1')

        


        
        #cmd_velocity_message.device_id = 3
        #cmd_velocity_message.value = 0.2
        #cmd_velocity_pub.publish(cmd_velocity_message)
        #time.sleep(3.0)
        #cmd_velocity_message.device_id = 3
        #cmd_velocity_message.value = -0.2
        #cmd_velocity_pub.publish(cmd_velocity_message)
        #time.sleep(3.0)




if __name__ == "__main__":
    rospy.init_node("keyboard_r5m")
    keyboard_r5m_main()
