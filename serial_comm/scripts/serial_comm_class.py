#!/usr/bin/env python
import packet_1
import packet_2
import packet_3
import packet_4
import packet_5
import packet_6
import packet_7

class Serial_Comm:
	# Constructor/initializer configures port buffers and msg type per node type
	def __init__(self, node_type):
		if node_type == 'mis_det_serial_node':
			self.reader = True
			self.pkt_in = packet_2.Packet_2()
			self.topic_out = 'mis/det_in_data'
			self.msg_out = self.pkt_in.msg
			self.writer = True
			self.pkt_out = packet_1.Packet_1()
			self.topic_in = 'mis/det_out_data'
			self.msg_in = self.pkt_out.msg
		elif node_type == 'det_mis_serial_node':
			self.reader = True
			self.pkt_in = packet_1.Packet_1()
			self.topic_out = 'det/mis_in_data'
			self.msg_out = self.pkt_in.msg
			self.writer = True
			self.pkt_out = packet_2.Packet_2()
			self.topic_in = 'det/mis_out_data'
			self.msg_in = self.pkt_out.msg
		elif node_type == 'mis_nb1_serial_node':
			self.reader = True
			self.pkt_in = packet_3.Packet_3()
			self.topic_out = 'mis/nb1_in_data'
			self.msg_out = self.pkt_in.msg
			self.writer = False
		elif node_type == 'mis_nb2_serial_node':
			self.reader = True
			self.pkt_in = packet_4.Packet_4()
			self.topic_out = 'mis/nb2_in_data'
			self.msg_out = self.pkt_in.msg
			self.writer = True
			self.pkt_out = packet_6.Packet_6()
			self.topic_in = 'mis/nb2_out_data'
			self.msg_in = self.pkt_out.msg
		elif node_type == 'mis_sib_serial_node':
			self.reader = True
			self.pkt_in = packet_5.Packet_5()
			self.topic_out = 'mis/sib_in_data'
			self.msg_out = self.pkt_in.msg
			self.writer = True
			self.pkt_out = packet_7.Packet_7()
			self.topic_in = 'mis/sib_out_data'
			self.msg_in = self.pkt_out.msg
#~~~add a default configuration with 'else'
		else:
			print "ERROR: Serial_Comm.__init__ --> No node type match."

