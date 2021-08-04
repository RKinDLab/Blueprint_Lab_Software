#!/usr/bin/env python

# Ignacio Carlucho and Corina Barbalata - March. 2020
#

""" MPC controller
    This node reads the position and velocity from the reach5 mini arm 
    and publishes torque outputs for the arm by performing an neural mpc controller
    optimized by means of the scipy minimize algorithm.
"""


import rospy
import numpy as np

import os 
from std_msgs.msg import Float32MultiArray, Int32, Float64MultiArray
from moveit_msgs.msg import MoveGroupActionGoal, RobotTrajectory
from std_srvs.srv import SetBool, Trigger, TriggerRequest, Empty
from blueprintlab_reachsystem_ros_messages.msg import single_float
import time

# from blueprintlab_reachsystem_ros_messages.msg import single_float


class mpc_controller(object):

    def __init__(self, ACTION_DIM = 4):
        """
        Initialization of the mpc controller. Most of the parameters are fixed as this depends on the arm we have.
        """
        # TODO: There is probably a more elegant way of doing this
        # load neural network params
        self.start = 0.

       
        if ACTION_DIM == 4: 
            self.ACTION_DIM = ACTION_DIM
        else: 
            print('you are doing something wrong')
            exit()
        # number of degress of freedom
        self.ndegres = 4
        self.state_dim = 2*self.ndegres
        # the horizon is hard coded
        self.horizon = 5
        # bounds
        # we start by a defining the bounds for each of the control inputs  
        self.lb = -8. # -220.
        self.up = 8. #  190. 
        self.current_lb = -250.
        self.current_up = 250.

        # initial action
        self.action = np.ones(self.ACTION_DIM) 
        # for the optimization problem we use a chain of actions
        # the initial estimate is zeros
        self.h_action = np.zeros(self.ACTION_DIM*self.horizon)
        self.start_time = []
        self._x = []
        self.pos_buffer = []
        self.vel_buffer =[]
        self.u_buffer = []
        self.error = np.zeros((3,4))
        # desired trajectory
        # TODO: This has to be moved to a subscriber so that the planner can send the commands
        self.waypoints = ([[0., 0., 0., 0., 0, 0., 0., 0. ]]) # ([[.1, 0., 0.05, 0., 0.5, 0., 0.2, 0. ], [.0, 0., 0., 0., 0.5, 0., 0.2, 0.]])
        self.flag = 0
        self.count = 0
        # ref is just a single command
        self.ref = self.waypoints[self.flag]
        # this takes into consideration the h_ref over the horizon 
        self.h_ref = np.array([self.ref for _ in range(self.horizon)])
        

        #TODO: subscribers
        self.nav_rate = 15
        self.dt = 1.0 / self.nav_rate
        self.timeout = 0.99*(1.0 / self.nav_rate) # it should time out earlier than 0.05 to keep the rate 
        self.r_loop = rospy.Rate(self.nav_rate)
        self.position = np.zeros(self.ndegres)
        self.unprocessded_position = np.zeros(self.ndegres)
        self.velocity = np.zeros(self.ndegres)
        self.state = np.zeros(self.ndegres)
        self.current_command = np.zeros(self.ndegres)
        # Publishers
        # we need integration with the arm for the tau
        #self.pub_tau = rospy.Publisher('/r5m_0/cmd_tau', Float32MultiArray, queue_size = 10)
        #self.pub_vel = rospy.Publisher('/r5m_0/cmd_velocity', Float32MultiArray, queue_size = 10)
        #self.pub_pos = rospy.Publisher('/r5m_0/cmd_position', Float32MultiArray, queue_size = 10)
        self.cmd_current_pub = rospy.Publisher('r5m_0/cmd_current', single_float, queue_size=10)
        # Subscribers
        self.sub_vel = rospy.Subscriber('/r5m_0/rec_velocity', Float32MultiArray, self.get_velocity, tcp_nodelay=True, queue_size=10)
        # self.sub_vel = rospy.Subscriber('/r5m_0/velocity', single_float, self.get_velocity_arm, tcp_nodelay=True, queue_size=10)
        # self.sub_pos = rospy.Subscriber('/r5m_0/rec_position', Float32MultiArray, self.get_position, tcp_nodelay=True, queue_size=10)
        self.sub_pln = rospy.Subscriber('/r5m_0/plan', RobotTrajectory, self.get_plan, tcp_nodelay=True, queue_size=10)
        self.sub_pos = rospy.Subscriber('/r5m_0/position', single_float, self.get_position_arm, tcp_nodelay=True, queue_size=10)
        # service
        self.req_joint_plan = rospy.ServiceProxy('/r5m_0/joint_plan_req', Empty)
        self.req_plan = rospy.ServiceProxy('/r5m_0/plan_req', Empty)

    def get_velocity(self, message):
        """
        velocity callback for the arm
        """
        #print('**************** vel ')
        self.velocity = message.data
        self.state[0:self.ndegres] = self.velocity[0:self.ndegres]

    def get_position(self, message):
        """
        position callback for the arm 
        """
        #print('**************** pos ')
        self.position = message.data
        self.state[self.ndegres:] = self.position[0:self.ndegres]

    def wrap_pi(self, angle):
        return ((angle+np.pi) % (2*np.pi)) - np.pi


    def get_position_arm(self, msg):

        # print(self.wrap_pi(msg.value))
        # pose = msg.value
        pose = msg.value
        # pose = self.wrap_pi(msg.value) 
        # pose = self.wrap_pi(msg.value) 
        if msg.device_id == 5:
            #print('0')
            # print('time **************** ', time.time() - self.start )
            # self.start = time.time()

            self.state[4] =  pose # 0.3*self.state[4] + 0.7*pose
            self.position[0] = pose # 0.3*self.state[4] + 0.7*pose
        elif msg.device_id == 4:
            #print('1')
            self.state[5] = pose
            self.position[1] = pose
        elif msg.device_id == 3:
            #print('2')
            self.state[6] = pose # 0.3*self.state[4] + 0.7*pose
            self.position[2] = pose # 0.3*self.state[4] + 0.7*pose
        elif msg.device_id == 2:
            #print('3')
            self.state[7] = pose
            self.position[3] = pose
        else:
            self.unprocessded_position[0] = msg.value 
        #print(self.state, self.position)

    def get_velocity_arm(self, msg):

        # print(self.wrap_pi(msg.value))
        # pose = msg.value
        # pose = msg.value
        # pose = msg.value
        # pose = self.wrap_pi(msg.value) 
        if msg.device_id == 5:
            #print('0')
            
            self.state[0] =  msg.value # 0.5*self.state[4] + 0.5*pose
            self.velocity[0] = msg.value # 0.5*self.state[4] + 0.5*pose
        elif msg.device_id == 4:
            #print('1')
            self.state[1] = msg.value
            self.velocity[1] = msg.value
        elif msg.device_id == 3:
            #print('2')
            self.state[2] = msg.value
            self.velocity[2] = msg.value
        elif msg.device_id == 2:
            #print('3')
            self.state[3] = msg.value
            self.velocity[3] = msg.value
        else:
            self.unprocessded_position[0] = msg.value 
        #print(self.state, self.position)



    def get_plan(self, message):
        """
        plan callback  
        """
        way_points = []
        self.flag = 0
        for i in range( len(message.joint_trajectory.points)):
            way_points.append(np.hstack([message.joint_trajectory.points[i].velocities, message.joint_trajectory.points[i].positions]))
            print(i, np.round(message.joint_trajectory.points[i].positions,3))
        self.waypoints = way_points
        print('waypoint', self.waypoints)
        print('apparently Ive got a plan')


    def reset(self):
        """
        Reset in case it is needed in the future
        """
        self.position = np.zeros(self.ndegres)
        self.velocity = np.zeros(self.ndegres)
        self.state = np.zeros(2*self.ndegres)
        self.flag = 0
        self.h_ref = np.array([self.ref for _ in range(self.horizon)])
        self.action = np.zeros(self.ACTION_DIM) 
        self.h_action = np.zeros(self.ACTION_DIM*self.horizon)



    def set_time(self):
        self.start_time = time.time()

    def controller_pid(self, et, et1, et2, ks, u0):
        
        Kp = ks[0]
        Ti = ks[1]
        Td = 0.

        k1 = Kp*(1+Td/self.dt)
        k2 =-Kp*(1+2*Td/self.dt-self.dt/Ti)
        k3 = Kp*(Td/Ti)

        u = u0/2. + k1*et + k2*et1 + k3*et2

        return u

    
    def P(self, ref):
        actions = np.zeros(4)

        kp = 2. 
        error1 = ref[4] - self.position[0] 
        error2 = ref[5] - self.position[1] 
        error3 = ref[6] - self.position[2] 
        actions[0] = kp*error1
        actions[1] = 3.*kp*error2
        actions[2] = 6.*kp*error3
        actions = np.clip(actions, self.lb, self.up)

        return actions 

    def PID(self, ref):
        k1 = np.array([1.,0.07,0.])
        k2 = np.array([2.5,0.05,0.])
        k3 = np.array([0.5,0.05,0.])
        k4 = np.array([2.,0.1,0.])

        self.error[2] = self.error[1]
        self.error[1] = self.error[0] 

        self.error[0][0] = ref[4] - self.position[0] 
        self.error[0][1] = ref[5] - self.position[1]
        self.error[0][2] = ref[6] - self.position[2]
        self.error[0][3] = ref[7] - self.position[3]
        
        actions = np.zeros(4)
        actions[0] = self.controller_pid(self.error[0][0], self.error[1][0], self.error[2][0], k1, self.action[0]) 
        actions[1] = self.controller_pid(self.error[0][1], self.error[1][1], self.error[2][1], k2, self.action[1]) 
        actions[2] = self.controller_pid(self.error[0][2], self.error[1][2], self.error[2][2], k3, self.action[2])
        actions[3] = self.controller_pid(self.error[0][3], self.error[1][3], self.error[2][3], k4, self.action[3])  

        #actions[0] = kp*error1
        #actions[1] = 3.*kp*error2
        #actions[2] = 6.*kp*error3
        actions = np.clip(actions, self.lb, self.up)

        return actions 




    def loop(self):    
        '''
        Main loop executes the controller and performs the optimization algorithm
        '''
        
        
        # rospy.wait_for_message('/r5m_0/plan', RobotTrajectory)
        #rospy.wait_for_service('/r5m_0/joint_plan_req')
        #rospy.wait_for_service('/r5m_0/plan_req')
        #print('requesting plan')
        #self.req_plan()
        # print('sleeping')
        

        rospy.sleep(2)
        '''
        print('***** Press: p to request plan or y to start controller and then enter *********')
        userinput = raw_input().lower()
        if (userinput[0] == 'p'):
            self.req_plan()
            rospy.sleep(0.5)
            print('***** Now press y to start controller *********')
            userinput = raw_input().lower()
            if (userinput[0] != 'y'):
                print('Incorrect input closing controller')
                exit()
        elif (userinput[0] == 'y'):
            print('Starting controller')
        else: 
            print('Incorrect input closing controller')
            exit()
        '''
        # self.flag = len(self.waypoints) -2 

        self.flag = 0
       
        # self.waypoints = np.array([[0.05, 0, 0, 0, 2.5, 1.59, 0.018, 2.98],[0., 0., 0., 0., 2.5, 1.59, 0.018, 2.98]])
        #self.waypoints = np.array([[0.1, 0, 0, 0, 2.3, 1.59, 0.018, 2.98],[0., 0., 0., 0., 2.5, 1.59, 0.018, 2.98]])
        # self.waypoints = np.array([[0., 0.05, 0.0, 0, 2.5, 2.0, 1.6, 2.98],[0., 0., 0., 0., 2.5, 2.0, 1.7, 2.98]])
        # HOME
        self.waypoints = np.array([[0.05, 0.05, 0.05, 0, 0.5, 1.4, 0.25, 0.3],[0., 0., 0., 0., 0.5, 1.4, 0.25, 0.3]])


        self.ref = self.waypoints[self.flag] 
    

        while not rospy.is_shutdown() and self.count < 250:

           
            


            # I do not need the horizon 
            # self.h_ref = np.array([self.ref for _ in range(self.horizon)])
            
            # actions are in torque
            self.action  = self.PID(self.ref)




            # some simple screep output 
            # print(self.flag, len(self.waypoints)-1, np.round(np.abs(np.linalg.norm(self.ref[4]-self.state[4])),3) ,'ref', np.round(self.ref[4:],3), 's', np.round(self.state[4:],3))
            print( 's', np.round(self.state[4:],3), 'a', np.round(self.action,3))

            # TODO: I am just switching thruogh the predefined reference command
            # if np.abs(np.linalg.norm(self.ref[4:]-self.state[4:]))<0.09:
            if np.abs(np.linalg.norm(self.ref[4]-self.state[4]))<0.05:
                if self.flag < len(self.waypoints)-1:
                    self.flag += 1 
                    self.ref = self.waypoints[self.flag]
                    self.count = 0


           
            # print(self.action)
            # Publish the action command over ROS
            # position and velocities are obtained from the callabacks 
            self.action_to_current()
            # print('send current')
            self.send_current()
            
            # this count is helpful for getting unstuck
            self.count += 1 
            self.buffer_store()

            # wait
            try:
                self.r_loop.sleep()
            except rospy.ROSInterruptException:
                #self.send_current_n(5,0)
                #print('sent zeros **************')
                pass
        self.send_current_n(5,0.)
        self.send_current_n(4,0.)
        self.send_current_n(3,0.)
        self.send_current_n(2,-90.)
        #self.send_current_n(1,0.)
        print('sent zeros **************')
        #self.show_results()

    def buffer_store(self):
        # print(np.array(self.position), self.velocity)

        self.pos_buffer.append(self.position.copy())
        self.vel_buffer.append(self.velocity)
        self.u_buffer.append(self.action)
        # print(self.pos_buffer)
        return 

    def action_to_current(self):
        # actions are torques 
        new_currents = np.zeros(self.action.shape)

        for i in range(4):
            # link 4 has a diferent torque dynamic
            if i ==3:
                if self.action[i]>0.:
                    new_currents[i] = (self.action[i]+0.4)/0.022
                else:
                    new_currents[i] = (self.action[i]-0.4)/0.022

            else: 
                if self.action[i]>0.:
                    new_currents[i] = (self.action[i]+0.4)/0.011
                else:
                    new_currents[i] = (self.action[i]-0.4)/0.011

        self.current_command = np.clip(new_currents, self.current_lb,  self.current_up)
        return 

    def send_current_(self):
        self.send_current_n(5, -200.)


    def send_current(self):
        # sends actions over ros
        # ms = Float32MultiArray()
        print('warning you are trying to send something', self.current_command)
        
        # Link 1
        if self.position[0] < 0.2:
            self.send_current_n(5,0.)
        else:
            #print('self.action 1 ',self.action)
            self.send_current_n(5,self.current_command[0])


        # Link 2

        

        # self.send_current_n(4,self.current_command[1])
        
        if self.position[1] < 1.1:
            self.send_current_n(4,0.)
        elif self.position[1] > 3.:
            self.send_current_n(4,0.)
        else:
            #print('self.action 2',self.action)
            self.send_current_n(4,self.current_command[1])
        
        

        # Link 3 
        # self.send_current_n(3,self.current_command[2])
        if self.position[2] < 0.2:
            self.send_current_n(3,0.)
        else:
            #print('self.action',self.action)
            self.send_current_n(3,self.current_command[2])

        
        # Link 4
        if self.position[3] < 0.2:
            self.send_current_n(2,0.)
        elif self.position[3] > 5.3:
            self.send_current_n(2,0.)
        else:
            # print('self.action',self.action)
            self.send_current_n(2,self.current_command[3])
        
        
        # self.send_current_n(5,0.)
        #self.pub_tau.publish(ms)

    def send_current_n(self, dev_id, current):
        current = np.clip(current,  self.current_lb, self.current_up)
        # print('warning you are trying to send something', current)
        cmd_current_message = single_float()
        cmd_current_message.device_id = dev_id
        cmd_current_message.value = current
        # print(cmd_current_message.device_id, cmd_current_message.value)
        # exit()
        self.cmd_current_pub.publish(cmd_current_message)
        return 
    
    # def send_velocity(self, value, device):
        # Sends velocities over ROS

        # TODO: fix manage multiple devices
        # print('warning you are trying to send something')
        #cmd_velocity_message.device_id = device
        #cmd_velocity_message.value = value
        #cmd_velocity_pub.publish(cmd_velocity_message)

    def show_results(self):
        import matplotlib.pyplot as plt
        
        np.save(self.dr + 'test_data_pos_buffer.npy', self.pos_buffer)
        np.save(self.dr + 'test_data_vel_buffer.npy', self.vel_buffer)
        np.save(self.dr + 'test_data_u_buffer.npy', self.u_buffer)

        single_u = np.zeros(len(self.u_buffer))
        for j in range(len(self.u_buffer)):
            single_u[j] = self.u_buffer[j][1]
            # single_pos[j] = self.pos_buffer[j][0]

        power = np.abs(single_u)*0.024
        energy = np.sum(power)*0.05
        print('energy', energy)
        plt.subplot(221)
        plt.plot(self.vel_buffer)
        plt.title('Velocities')
        plt.legend(['1', '2', '3', '4'])
        plt.subplot(222)
        plt.plot(self.pos_buffer)
        plt.title('Position')
        plt.legend(['1', '2', '3', '4'])
        plt.subplot(223)
        plt.plot(single_u)
        plt.title('Actions')
        plt.legend(['1', '2', '3', '4'])
        plt.subplot(224)
        plt.plot(power)
        plt.title('Power [W]')
        # plt.legend(['1', '2', '3', '4'])
        plt.savefig(self.dr +'nn_mpc_action.png', dpi=600)
        plt.show()








if __name__ == "__main__":
    rospy.init_node("mpc_controller")
    mpc = mpc_controller(4)
    mpc.reset()
    mpc.loop()
