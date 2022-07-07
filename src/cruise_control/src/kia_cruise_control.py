#!/usr/bin/python

import rospy
from std_msgs.msg import Float32
import pandas as pd
import numpy as np
from pid import PID
import time as t
import matplotlib.pyplot as plt

from roscco.msg import EnableDisable, ThrottleCommand, BrakeCommand

# change path to recorded cycle
# df = pd.read_csv('~/kia_drive_cycles.csv')

# target_speeds = df['velocity'] # get speeds from drive cycle
# time = df['time'] # get time from drive cycle

df = pd.read_csv('uddscol.txt' , delimiter='\t')

df['Time'] = df['Time'].astype('float')
df['Speed'] = df['Speed'].astype('float')

#df = df[ df['Time'] <= 400 ]
# df = df[ df['Time'] >= 20 ]

# df['Time'] = df['Time'] - df['Time'][20]
# df = df.reset_index()

time = df['Time']
target_speeds = df['Speed']/2.237

def commands(ysat):
    '''
        params:
            -Input: y has to be clipped between -1 and 1
            -Output: throttle and brake commands
    '''

    assert -1 < ysat < 1

    if ysat < 0:
        throttle = 0.0
        if -1.0 <= ysat <= 0.0:
            brake = -ysat
        else:
            brake = 0.8 # full brake
    elif 0.0 <= ysat <= 1.0:
        throttle = ysat
        brake = 0.0
    else: # ysat > 1.0
        throttle = 1.0
        brake = 0.0

    return np.clip(throttle, 0.0, 1.0), np.clip(brake, 0.0, 1.0)

class NODE():

    def __init__(self):

        self.v = 0.0
        self.velocity = []
        self.time = []

        rospy.init_node('drive_cycle_control_node')
        rospy.Subscriber('can_frame_speed', Float32, self.callback)

        self.enab = rospy.Publisher('/enable_disable', EnableDisable, queue_size=10)
        self.thr_pub = rospy.Publisher('throttle_command', ThrottleCommand, queue_size=10)
        self.brk_pub = rospy.Publisher('brake_command', BrakeCommand, queue_size=10)

        self.rate = rospy.Rate(20) # publish at 20 Hz

        self.run_step() # initiate run step
    
    def search_speed(self, t, tarray):
        
        '''
            This function searches the time in the recorded drive cycle

            params:
                t - current time since started simulation
                tarray - recorded array time

            return:
                index of closest time
        '''

        d = [abs(t - icx) for icx in tarray]

        target_idx = np.argmin(d)

        return target_idx

    def callback(self, msg):

        '''
            Callback to get speed from ROS topic

        '''
        
        self.v = msg.data

    def run_step(self):

        msg = EnableDisable()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'kia'
        msg.enable_control = True

        pid = PID(0.3, 0.05, 0.0)
        pid.set_limits(1.0, -1.0) # upper, lower bound

        # throttle_pid = PID(0.3, 0.1, 0.05)
        # throttle_pid.set_limits(0.5, 0.0)

        # brake_pid = PID(0.05, 0.01, 0.05)
        # brake_pid.set_limits(1.0, 0.0)

        start = t.time()
        ti = rospy.Time.now().to_sec() # start time for PID dt

        thr_msg = ThrottleCommand()
        brk_msg = BrakeCommand()
        brk_msg.header.frame_id = 'kia'
        thr_msg.header.frame_id = 'kia'

        # self.enab.publish(msg)

        # thr_msg.throttle_position = 0.0
        # self.thr_pub.publish(thr_msg)

        while not rospy.is_shutdown():

            #self.enab.publish(msg)

            end = t.time()
            elapsed_time = end - start # elapsed time to find speed associated with speed

            idx = self.search_speed(elapsed_time, time)

            if idx >= ( len(time) - 5):

                rospy.signal_shutdown("Done")

            else:
                target_speed = target_speeds[idx] # get target speed
                target_acceleration = ( target_speeds[idx+1]-target_speeds[idx] )/ (time[idx+1] - time[idx])
                
                error = target_speed - self.v # target error

                self.velocity.append(self.v)
                self.time.append(elapsed_time)

                tf = rospy.Time.now().to_sec()
                dt = tf - ti # PID dt
                ti = tf # reset timer

                if dt == 0.0:
                    dt = 0.05

                y = pid.control(error, dt)
                throttle, brake = commands( np.clip( y, -1.0, 1.0 ) )
                #throttle = throttle_pid.control(error, dt)
                #throttle = throttle_pid(error)

                brk_msg.header.stamp = rospy.Time.now()
                thr_msg.header.stamp = rospy.Time.now()

                thr_msg.throttle_position = throttle
                self.thr_pub.publish(thr_msg)

                brk_msg.brake_position = brake
                self.brk_pub.publish(brk_msg)

                print("Throttle: {}, Brake: {}".format(throttle, brake))
                # print("Target speed: {}, Current speed: {}, Throttle: {}".format(target_speed, self.v, throttle))

                self.rate.sleep() # sleep for timer

    # def __del__(self):
    #     plt.plot( time, target_speeds , label="Drive cycle")
    #     plt.plot(self.time, self.velocity, label="Kia Soul")
    #     plt.xlabel('Time (s)')
    #     plt.ylabel('Velocity (mps)')
    #     plt.title('Velocity vs time', fontweight='bold')
    #     plt.legend()
    #     plt.grid()
    #     plt.show()

try:
    obj = NODE()
except rospy.ROSInterruptException:
    pass

# del obj