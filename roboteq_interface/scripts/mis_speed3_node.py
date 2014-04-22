#!/usr/bin/env python
import rospy
from roboteq_interface.msg import motor_commands
from roboteq_interface.msg import speed3_data
import serial
import struct
import time

callback_time = 0.0

def openPort():
	global port_open
	global ser
	echof_plus = False
	echof_timeout = 0.1
	ser=serial.Serial("/dev/ttyACM2",115200,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,0,False,False,None,False,None)
	ser.flushInput()
	ser.flushOutput()
	#rospy.loginfo("Opened Controller 3")
	while not echof_plus:
		ser.write("^ECHOF 1\r")
		n = readChar(ser,1.0)
		initial_time = time.clock()
		current_time = time.clock()
		while (not checkChar(n,b'\x2B')) and ((current_time - initial_time) <= echof_timeout):
			#rospy.loginfo("Looping for ECHOF +")
			n = readChar(ser,1.0)
			if checkChar(n,b'\x2B'):
				echof_plus = True
			current_time = time.clock()
	n = readChar(ser,1.0)
	while not checkChar(n,b'\x0D'):
		n = readChar(ser,1.0)
		#rospy.loginfo("Looping for ECHOF CR")
	#rospy.loginfo("Read ECHOF + and CR")
	port_open = True

def callback(msg_in):
	global motor_1_speed
	global motor_2_speed
	global callback_time
	motor_1_speed = msg_in.cont_3_motor_1_speed_cmd
	motor_2_speed = msg_in.cont_3_motor_2_speed_cmd
	callback_time = time.clock()

def mis_speed3_node():
	global ser
	global motor_1_speed
	global motor_2_speed
	global encoder_1_sign
	global encoder_2_sign
	global RPM_1_sign
	global RPM_2_sign
	global port_open
	global callback_time
	pub = rospy.Publisher('mis/speed3_data', speed3_data)
	sub = rospy.Subscriber('mis/motor_commands',motor_commands,callback)
	rospy.init_node('mis_speed3_node')
	msg_out = speed3_data()
	motor_1_speed = 0
	motor_2_speed = 0
	callback_timeout = 1.0
	p = ''
	port_open = False
	while not port_open:
		try:
			openPort()
		except serial.SerialException:
			#rospy.loginfo("Controller 3 not open")
			pass
	while not rospy.is_shutdown():
		current_time = time.clock()
		if (current_time - callback_time) >= callback_timeout:
			motor_1_speed = 0
			motor_2_speed = 0
		out_buff1 = "!G 1 "+str(motor_1_speed)+"\r"
		ser.write(out_buff1)
		p = readChar(ser,1.0)
		while p != b'\x2B':
			p = readChar(ser,1.0)
			#rospy.loginfo("Looping for first +")
		p = readChar(ser,1.0)
		while p != b'\x0D':
			p = readChar(ser,1.0)
			#rospy.loginfo("Looping for first CR")
		out_buff2 = "!G 2 "+str(motor_2_speed)+"\r"
		ser.write(out_buff2)
		p = readChar(ser,1.0)
		while p != b'\x2B':
			p = readChar(ser,1.0)
			#rospy.loginfo("Looping for second +")
		p = readChar(ser,1.0)
		while p != b'\x0D':
			p = readChar(ser,1.0)
			#rospy.loginfo("Looping for first CR")
		ser.write("?C\r") # Request Absolute Encoder Counts
		m=readChar(ser,1.0)
		#rospy.loginfo("Looking for C, Actually Read %d",ord(m))
		if checkChar(m,b'\x43') == True:
			#rospy.loginfo("Read C")
			m=readChar(ser,0.1)
			#rospy.loginfo(m)
			if checkChar(m,b'\x3D') == True:
				#rospy.loginfo("Read =")
				i=0
				reading = True
				motor_1_encoder_count = 0
				motor_2_encoder_count = 0
				read_second_encoder = False # False means motor channel 1; True means motor channel 2
				encoder_1_sign = 1.0
				encoder_2_sign = 1.0
				while reading == True:
					m=readChar(ser,0.1)
					if m == "\r":
						#rospy.loginfo("Read end of line")
						reading = False
					else:
						if m == b'\x3A':
							#rospy.loginfo("Read colon")
							read_second_encoder = True
							m=readChar(ser,0.1)
						if read_second_encoder == False:
							if m == b'\x2D':
								#rospy.loginfo("Read minus")
								encoder_1_sign = -1.0
								m=readChar(ser,0.1)
							motor_1_encoder_count = motor_1_encoder_count*10+int(m)
							reading = True
							read_second_encoder = False
						else:
							if m == b'\x2D':
								#rospy.loginfo("Read minus")
								encoder_2_sign = -1.0
								m=readChar(ser,0.1)
							motor_2_encoder_count = motor_2_encoder_count*10+int(m)
							reading = True
							read_second_encoder = True
				msg_out.motor_1_encoder_count = motor_1_encoder_count*encoder_1_sign
				msg_out.motor_2_encoder_count = motor_2_encoder_count*encoder_2_sign
		ser.write("?S\r") # Request RPM
		m=readChar(ser,1.0)
		if checkChar(m,b'\x53'):
			#rospy.loginfo("Read S")
			m=readChar(ser,0.1)
			#rospy.loginfo(m)
			if checkChar(m,b'\x3D'):
				#rospy.loginfo("Read =")
				i=0
				reading = True
				motor_1_RPM = 0
				motor_2_RPM = 0
				read_second_RPM = False # False means motor channel 1; True means motor channel 2
				RPM_1_sign = 1.0
				RPM_2_sign = 1.0
				while reading == True:
					m=readChar(ser,0.1)
					if m == "\r":
						#rospy.loginfo("Read end of line")
						reading = False
					else:
						if m == b'\x3A':
							#rospy.loginfo("Read colon")
							read_second_RPM = True
							m=readChar(ser,0.1)
						if read_second_RPM == False:
							if m == b'\x2D':
								#rospy.loginfo("Read minus")
								RPM_1_sign = -1.0
								m=readChar(ser,0.1)
							motor_1_RPM = motor_1_RPM*10+int(m)
							reading = True
							read_second_RPM = False
						else:
							if m == b'\x2D':
								#rospy.loginfo("Read minus")
								RPM_2_sign = -1.0
								m=readChar(ser,0.1)
							motor_2_RPM = motor_2_RPM*10+int(m)
							reading = True
							read_second_RPM = True
				msg_out.motor_1_RPM = motor_1_RPM*RPM_1_sign
				msg_out.motor_2_RPM = motor_2_RPM*RPM_2_sign
		pub.publish(msg_out)
		prior_time = time.clock()
		r = rospy.Rate(50)
		r.sleep()

def readChar(ser,timeout_time):
	initial_time = time.clock()
	timeout = False
	m = ser.read(1)
	while (len(m) == 0) and (timeout == False):
		m = ser.read(1)
		current_time = time.clock()
		if (current_time - initial_time) >= timeout_time:
			timeout = True
		else:
			timeout = False
	return m
	
def checkChar(m,desired_char_hex):
	if m == desired_char_hex:
		return True
	else:
		return False

if __name__ == '__main__':
    try:
        mis_speed3_node()
    except rospy.ROSInterruptException:
        pass
