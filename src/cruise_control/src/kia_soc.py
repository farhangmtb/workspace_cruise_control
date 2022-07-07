#!/usr/bin/env python3

#Basic code for pulling SOC from Kia Niro and Soul
#This code requires that a Kvaser/socketcan interface is set up

import time
import can
import rospy
import numpy as np
from tqdm import tqdm
import struct

from std_msgs.msg import Float32

bustype = 'socketcan'
channel = 'can1'
filters = [{"can_id": 0x10, "can_mask": 0xFFFFFFF},
			{"can_id": 0x7EC, "can_mask": 0xFFFFFFF}]

#SOC request message - returns multiframe message with many parameters from BMS
bms_request=can.Message(is_extended_id=False, arbitration_id=2020,
	data=bytearray([2,33,1,0,0,0,0,0]))

#Flow control frame - specifies the format for the subsequent multiframe message
flow_control_frame=can.Message(is_extended_id=False, arbitration_id=2020,
	data=bytearray([48,8,2,0,0,0,0,0]))

"""
	Code for pulling the SOC signal:
	The signal is in data byte 8 of the multiframe message.
	The sequence is:

	1. Send request (bms_request) on hex 7E4 (dec 2020)
	2. Wait for a message starting with 10 (16) on arbid 7EC (2028)
	3. Send flow control frame (flow_control_frame) on 7E4 (2020)
	4. Wait for a message starting with 21 (33) on 7EC (2028)
"""

class SocCan(object):

	'''
		TODO: bus.receive doesnt guarantee that I will 
		recieve message that I am looking for
	'''

	def __init__(self, channel, bustype, filters, request_id = 2028,
		request_byte = 16, message_id = 33, byte_size = 61, message_byte = 14):
		'''
			Construct two buses:
			 1. for sending request
			 2. for receiving info (since it blocks thread), in order to avoid iter
		'''

		self.v = 0.0
		
		rospy.init_node("soc_node")
		
		self.request = can.Bus(channel=channel,interface=bustype)
		self.bus = can.Bus(channel=channel,interface=bustype, can_filters=filters)

		rospy.Subscriber('/can_frame_speed', Float32, self.speed_callback)
		self.pub = rospy.Publisher('can_frame/soc', Float32, queue_size=10)
		self.speed_pub = rospy.Publisher('vehicle/speed', Float32, queue_size=10)

		self.rate = rospy.Rate(100)

		self.periodic_update(request_id, request_byte, message_id, byte_size, message_byte)

	def speed_callback(self, msg):
		self.v = msg.data
	
	def multiframe_search(self, msg_data, sum, message_byte, byte_size, first):
		
		if first:
			sum += len( msg_data[2:] )
			first = False

		else:
			sum += len( msg_data[1:] )

		if sum > message_byte:
			i = 8 - ( sum - message_byte - 1 )
			
			self.pub.publish( float( msg_data[i]/2 ) )

			return 1, sum, first

		elif sum > byte_size:
			raise ValueError("Multiframe byte size exceeded")

		else:
			return 0, sum, first

	
	def periodic_update(self, request_id, request_byte, 
					message_id, byte_size, message_byte):

		'''
			Periodic update to get SOC every t seconds
		'''

		while not rospy.is_shutdown():

			rospy.loginfo("Sent request message")

			# send request
			self.request.send(bms_request)

			# waiting for response from request
			msg = self.bus.recv()

			# wait for response from sent request
			if msg.data[0] == request_byte:

				rospy.loginfo("Received vehicle response")

				rospy.loginfo("Sending flow control")

				# send flow control frame
				self.request.send(flow_control_frame)

				sum = 0
				first = True

				while True:

					# wait for message to come in
					msg = self.bus.recv()

					if msg.data[0] == message_id:

						rospy.loginfo("1st multiframe message found")

						rospy.loginfo("Collecting SOC")

						# get SOC
						#print( "SOC: {}, Current: {}".format( msg.data[1]/2, msg.data[7]) )
						self.pub.publish( float(msg.data[1])/2 )
						self.speed_pub.publish( self.v )

					if msg.data[0] == 34:

						rospy.loginfo("2nd multiframe message found")

						rospy.loginfo("Collecting Voltage")

						# get voltage
						#print( "Voltage: {}".format( struct.unpack('h', msg.data[1:3])[0]/10 ) )
						
						break

					self.rate.sleep()		

try:
	SocCan(channel=channel, bustype=bustype, filters=filters)
except rospy.ROSInterruptException:
	pass
