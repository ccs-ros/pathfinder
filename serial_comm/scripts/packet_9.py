from ctypes import *

class packet_9_type(Structure):
	_pack_ = 1
	_fields_ = [("counter",c_ushort),("time",c_ushort),("motor_servo",c_short),("LED",c_ubyte),("status",c_ubyte),("reserved",c_ubyte*18),("checksum",c_ubyte)]

class packet_9_buff(Union):
	_fields_ = [("char_buff",c_ubyte*31),("packet_buff",packet_9_type)]
