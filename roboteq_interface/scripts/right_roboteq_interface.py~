#!/usr/bin/env python
import rospy
from roboteq_interface.msg import motor_speed_control
from roboteq_interface.msg import right_motor_encoders
import serial
import struct
import time

def openPort():
	global port_open
	global ser
	ser=serial.Serial("/dev/ttyACM1",115200,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,0,False,False,None,False,None)
	ser.flushInput()
	ser.flushOutput()
	port_open = True
	ser.write("^ECHOF 1\r")
	n = ser.read()
	while not n == b'\x2B':
		rospy.loginfo("Looping for ECHOF +")
		n = ser.read()
	rospy.loginfo("Read ECHOF +")

def callback(msg_in):
	global motor_1_encoder_count
	global motor_1_speed
	global motor_2_speed
	motor_1_speed = msg_in.cont_2_motor_1_speed_cmd
	motor_2_speed = msg_in.cont_2_motor_2_speed_cmd

def right_roboteq_interface():
	global ser
	global motor_1_speed
	global motor_2_speed
	global encoder_1_sign
	global encoder_2_sign
	global port_open
	pub = rospy.Publisher('right_roboteq_encoders', right_motor_encoders)
	sub = rospy.Subscriber('motor_speed_control',motor_speed_control,callback)
	rospy.init_node('right_roboteq_interface')
	msg_out = right_motor_encoders()
	motor_1_speed = 0
	motor_2_speed = 0
	p = ''
	port_open = False
	while not port_open:
		try:
			openPort()
		except serial.SerialException:
			rospy.loginfo("Right motor not open")
	while not rospy.is_shutdown():
		out_buff1 = "!G 1 "+str(motor_1_speed)+"\r"
		ser.write(out_buff1)
		out_buff2 = "!G 2 "+str(motor_2_speed)+"\r"
		ser.write(out_buff2)
		p = ser.read()
		while not p == b'\x2B':
			rospy.loginfo("Looping for first +")
			p = ser.read()
		rospy.loginfo("Read first +")
		p = ser.read()
		while not p == b'\x2B':
			rospy.loginfo("Looping for second +")
			p = ser.read()
		rospy.loginfo("Read second +")
		ser.write("?C\r")
		m=ser.read()
		while not m == b'\x43':
			#rospy.loginfo("Looping for C")
			m=ser.read()
			rospy.loginfo(m)
		rospy.loginfo("Read C")
		m=ser.read()
		rospy.loginfo(m)
		if m==b'\x3D':
			rospy.loginfo("Read =")
			i=0
			reading = True
			motor_1_encoder_count = 0
			motor_2_encoder_count = 0
			read_second_encoder = False # False means motor channel 1; True means motor channel 2
			encoder_1_sign = 1.0
			encoder_2_sign = 1.0
			while reading == True:
				m=ser.read(1)
				if m == "\r":
					rospy.loginfo("Read end of line")
					reading = False
				else:
					if m == b'\x3A':
						rospy.loginfo("Read colon")
						read_second_encoder = True
						m=ser.read(1)
					if read_second_encoder == False:
						if m == b'\x2D':
							rospy.loginfo("Read minus")
							encoder_1_sign = -1.0
							m=ser.read(1)
						motor_1_encoder_count = motor_1_encoder_count*10+int(m)
						reading = True
						read_second_encoder = False
					else:
						if m == b'\x2D':
							rospy.loginfo("Read minus")
							encoder_2_sign = -1.0
							m=ser.read(1)
						motor_2_encoder_count = motor_2_encoder_count*10+int(m)
						reading = True
						read_second_encoder = True
			msg_out.motor_1_encoder_count = motor_1_encoder_count*encoder_1_sign
			msg_out.motor_2_encoder_count = motor_2_encoder_count*encoder_2_sign
			pub.publish(msg_out)
		prior_time = time.clock()
		r = rospy.Rate(50)
		r.sleep()


if __name__ == '__main__':
    try:
        right_roboteq_interface()
    except rospy.ROSInterruptException:
        pass
