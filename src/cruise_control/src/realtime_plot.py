#!/usr/bin/env python3

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib import animation
import rospy
from std_msgs.msg import Float32
import time

df = pd.read_csv('uddscol.txt' , delimiter='\t')

df['Time'] = df['Time'].astype('float')
df['Speed'] = df['Speed'].astype('float')

#df = df[ df['Time'] <= 400 ]
# df = df[ df['Time'] >= 20 ]

# df['Time'] = df['Time'] - df['Time'][20]
# df = df.reset_index()

t = df['Time']
target_speeds = df['Speed']/2.237

class NODE(object):

    def __init__(self):
        rospy.init_node('live_plot_node')

        self.x_vals, self.y_vals = [], []
        self.v = 0.0
        self.start_time = time.time()
        self.end_time = 0.0

        rospy.Subscriber('/can_frame_speed', Float32, self.callback)

        anim = animation.FuncAnimation(plt.gcf(), self.animate, interval=50)

        plt.show()

        rospy.spin()

    def callback(self, msg):
        self.end_time = time.time()
        self.v = msg.data

    def animate(self, i):

        self.x_vals.append(self.end_time - self.start_time)
        self.y_vals.append(self.v)

        plt.cla()
        plt.plot(t, target_speeds)
        plt.plot(self.x_vals, self.y_vals)
        plt.tight_layout()

try:
    NODE()
except KeyboardInterrupt:
    pass