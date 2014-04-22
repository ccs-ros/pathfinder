#!/usr/bin/env python
#import packet_1
#import pkt_1_msg
#import packet_2
#import pkt_2_msg
#import packet_3
#import pkt_3_msg
#import packet_4
#import pkt_4_msg
#import packet_5
#import pkt_5_msg
#import packet_6
#import pkt_6_msg
#import packet_7
#import pkt_7_msg
import packet_8
import pkt_8_msg
import packet_9
import pkt_9_msg

class serial_comm:
	# Constructor configures port buffers and msg type per node type
	def __init__(self, node_type):
		if node_type == 'mis_nav':
#~~~			pub = rospy.Publisher('nav_in_data',packet1)
#~~~			sub = rospy.Subscriber('nav_out_data',packet3)
			self.reader = True
			self.pkt_in = packet_1()
			self.topic_out = 'nav_in_data'
			self.msg_out = pkt_1_msg()
			self.writer = True
			self.topic_in = 'nav_out_data'
			self.msg_in = pkt_3_msg()
			self.pkt_out = packet_3()
		else if node_type == 'mis_det':
#~~~			pub = rospy.Publisher('det_in_data',packet2)
#~~~			sub = rospy.Subscriber('det_out_data',packet3)
			self.reader = True
			self.pkt_in = packet_2()
			self.topic_out = 'det_in_data'
			self.msg_out = pkt_2_msg()
			self.writer = True
			self.topic_in = 'det_out_data'
			self.msg_in = pkt_3_msg()
			self.pkt_out = packet_3()
		else if node_type == 'nav_mis':
#~~~			pub = rospy.Publisher('mis_in_data',packet3)
#~~~			sub = rospy.Subscriber('mis_out_data',packet1)
			self.reader = True
			self.pkt_in = packet_3()
			self.topic_out = 'mis_in_data'
			self.msg_out = pkt_3_msg()
			self.writer = True
			self.topic_in = 'mis_out_data'
			self.msg_in = pkt_1_msg()
			self.pkt_out = packet_1()
		else if node_type == 'nav_det':
#~~~			sub = rospy.Subscriber('det_out_data',packet1)
			self.reader = False
			self.writer = True
			self.topic_in = 'det_out_data'
			self.msg_in = pkt_1_msg()
			self.pkt_out = packet_1()
		else if node_type == 'det_mis':
#~~~			pub = rospy.Publisher('mis_in_data',packet3)
#~~~			sub = rospy.Subscriber('mis_out_data',packet2)
			self.reader = True
			self.pkt_in = packet_3()
			self.topic_out = 'mis_in_data'
			self.msg_out = pkt_3_msg()
			self.writer = True
			self.topic_in = 'mis_out_data'
			self.msg_in = pkt_2_msg()
			self.pkt_out = packet_2()
		else if node_type == 'nav_NB1':
#~~~			pub = rospy.Publisher('NB1_in_data',packet4)
			self.reader = True
			self.pkt_in = packet_4()
			self.topic_out = 'NB1_in_data'
			self.msg_out = pkt_4_msg()
			self.writer = False
		else if node_type == 'nav_NB2':
#~~~			pub rospy.Publisher('NB2_in_data',packet5)
			self.reader = True
			self.pkt_in = packet_5()
			self.topic_out = 'NB2_in_data'
			self.msg_out = pkt_5_msg()
			self.writer = False
		else if node_type == 'nav_sib':
			self.reader = True
			self.pkt_in = packet_6()
			self.topic_out = 'sib_in_data'
			self.msg_out = pkt_6_msg()
			self.writer = True
			self.topic_in = 'sib_out_data'
			self.msg_in = pkt_7_msg()
			self.pkt_out = packet_7()
		else if node_type == 'mis_sib':
			self.reader = True
			self.pkt_in = packet_8()
			self.topic_out = 'sib_in_data'
			self.msg_out = pkt_8_msg()
			self.writer = True
			self.topic_in = 'sib_out_data'
			self.msg_in = pkt_9_msg()
			self.pkt_out = packet_9()


