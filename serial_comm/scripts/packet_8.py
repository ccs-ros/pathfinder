from ctypes import *
import packet_class
import pkt_8_msg

class packet_8:

	

	class packet_8_type(Structure):
		_pack_ = 1
		_fields_ = [("counter",c_ushort),("time",c_ushort),("acceleration_x",c_short),("acceleration_y",c_short),("acceleration_z",c_short),("rate_p",c_short),("rate_q",c_short),("rate_r",c_short),("A2D_1",c_short),("A2D_2",c_short),("A2D_3",c_short),("A2D_4",c_short),("A2D_5",c_short),("A2D_6",c_short),("A2D_7",c_short),("A2D_8",c_short),("laser_1",c_ushort),("laser_2",c_ushort),("speed_controller_1",c_short),("speed_controller_2",c_short),("speed_controller_3",c_short),("speed_controller_4",c_short),("switches",c_ubyte),("voltage",c_ubyte),("status",c_ubyte),("reserved",c_ubyte*12),("checksum",c_ubyte)]

	class packet_8_buff(Union):
		_fields_ = [("char_buff",c_ubyte*61),("packet",packet_8_type)]

	def unpack_buffer2msg(self,msg):
		msg.counter = self.packet.counter
		msg.time = self.packet.time
		msg.acceleration_x = self.packet.acceleration_x
		msg.acceleration_y = self.packet.acceleration_y
		msg.acceleration_z = self.packet.acceleration_z
		msg.rate_p = self.packet.rate_p
		msg.rate_q = self.packet.rate_q
		msg.rate_r = self.packet.rate_r
		msg.A2D_1 = self.packet.A2D_1
		msg.A2D_2 = self.packet.A2D_2
		msg.A2D_3 = self.packet.A2D_3
		msg.A2D_4 = self.packet.A2D_4
		msg.A2D_5 = self.packet.A2D_5
		msg.A2D_6 = self.packet.A2D_6
		msg.A2D_7 = self.packet.A2D_7
		msg.A2D_8 = self.packet.A2D_8
		msg.laser_1 = self.packet.laser_1
		msg.laser_2 = self.packet.laser_2
		msg.speed_controller_1 = self.packet.speed_controller_1
		msg.speed_controller_2 = self.packet.speed_controller_2
		msg.speed_controller_3 = self.packet.speed_controller_3
		msg.speed_controller_4 = self.packet.speed_controller_4
		msg.switches = self.packet.switches
		msg.voltage = self.packet.voltage
		msg.status = self.packet.status

	def pack_msg2buffer(self):
		self.packet.counter = msg.counter
		self.packet.time = msg.time
		self.packet.acceleration_x = msg.acceleration_x
		self.packet.acceleration_y = msg.acceleration_y
		self.packet.acceleration_z = msg.acceleration_z
		self.packet.rate_p = msg.rate_p
		self.packet.rate_q = msg.rate_q
		self.packet.rate_r = msg.rate_r
		self.packet.A2D_1 = msg.A2D_1
		self.packet.A2D_2 = msg.A2D_2
		self.packet.A2D_3 = msg.A2D_3
		self.packet.A2D_4 = msg.A2D_4
		self.packet.A2D_5 = msg.A2D_5
		self.packet.A2D_6 = msg.A2D_6
		self.packet.A2D_7 = msg.A2D_7
		self.packet.A2D_8 = msg.A2D_8
		self.packet.laser_1 = msg.laser_1
		self.packet.laser_2 = msg.laser_2
		self.packet.speed_controller_1 = msg.speed_controller_1
		self.packet.speed_controller_2 = msg.speed_controller_2
		self.packet.speed_controller_3 = msg.speed_controller_3
		self.packet.speed_controller_4 = msg.speed_controller_4
		self.packet.switches = msg.switches
		self.packet.voltage = msg.voltage
		self.packet.status = msg.status

