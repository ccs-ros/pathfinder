from ctypes import *

class packet_class:

	class header_type(Structure):
		_pack_ = 1
		_fields_ = [("H1",c_ubyte),("H2",c_ubyte),("H3",c_ubyte)]

	class header_union(Union):
		_fields_ = [("header_buff",c_ubyte*3),("header",header_type)]

	def __init__(self)
		self.H1 = 'A'
		self.H2 = 'z'
