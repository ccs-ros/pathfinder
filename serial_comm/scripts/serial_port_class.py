#!/usr/bin/env python
import rospy
import serial
import time
import serial_comm_class

class serial_port:
	# Constructor opens and flushes serial port
	#~~~ determine how to read from parameter server
	def __init__(self):
		port_path = rospy.get_param('port_path', '/dev/ttyS0')
		self.ser = serial.Serial(port_path,115200,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,0,False,False,None,False,None)
		self.ser.flushInput()
		self.ser.flushOutput()
	
	# run() implements the main loop for the node
	def run(self):
		node_type = rospy.get_param('node_type', 'mis_sib')
		comm = serial_comm(node_type)
		rospy.init_node(node_type)
		#~~~ Must pub second parameter be a type, or is an object ok?
		if (comm.reader):
			pub = rospy.Publisher(comm.topic_out, comm.msg_out) 
		#~~~ Same question about sub second parameter
		if (comm.writer):
			sub = rospy.Subscriber(comm.topic_in, comm.msg_in, comm.pkt_out.pack_buffer)

#		if ***PARAM*** == 'mis_nav':
#			rospy.init_node('')
#			pkt = 
#			pub = rospy.Publisher('nav_in_data',packet1)
#			sub = rospy.Subscriber('nav_out_data',packet3)
#		else if ***PARAM*** == 'mis_det':
#			pub = rospy.Publisher('det_in_data',packet2)
#			sub = rospy.Subscriber('det_out_data',packet3)
#		else if ***PARAM*** == 'nav_mis':
#			pub = rospy.Publisher('mis_in_data',packet3)
#			sub = rospy.Subscriber('mis_out_data',packet1)
#		else if ***PARAM*** == 'nav_det':
#			sub = rospy.Subscriber('det_out_data',packet1)
#		else if ***PARAM*** == 'det_mis':
#			pub = rospy.Publisher('mis_in_data',packet3)
#			sub = rospy.Subscriber('mis_out_data',packet2)
#		else if ***PARAM*** == 'nav_NB1':
#			pub = rospy.Publisher('NB1_in_data',packet4)
#		else if ***PARAM*** == 'nav_NB2':
#			pub rospy.Publisher('NB2_in_data',packet5)


		prior_time = time.clock()
		
		# read new packet from serial port
		while not rospy.is_shutdown():
			if (comm.reader):
				m=ser.read()
				if m==comm.pkt_in.H1:
					m=ser.read()
					if m==comm.pkt_in.H2:
						m=ser.read()
						if m==comm.pkt_in.H3:
   	             		i=0
							while i<comm.pkt_in.size - 3:
								m=ser.read(1)
								if len(m) != 0:
									n=struct.unpack("B",m)
								comm.pkt_in.char_buff[i] = n[0]
								i=i+1

							# transfer data from serial packet to ROS msg and publish
							comm.pkt_in.unpack_buffer2msg(comm.msg_out)
							pub.publish(comm.msg_out)

			# check for serial output time and write to the port
			current_time = time.clock()	
			if (comm.writer):
				if current_time - prior_time >= 1/***CPARAM***comm_node.pkt_out.rate:  # rate in Hz
					#~~~check on + for string concatenate
					ser.write(comm.pkt_out.header_buff+comm.pkt_out.buffer)
			prior_time = time.clock()
			
			#~~~should this be the input packet rate?
			rospy.Rate(***CPARAM***comm_node.pkt_out.rate*2)

# launch run() for main node loop
if __name__ == '__main__':
    try:
        port = serial_port()
		port.run()
    except rospy.ROSInterruptException:
        pass
					
