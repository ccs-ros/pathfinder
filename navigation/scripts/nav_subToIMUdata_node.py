#!/usr/bin/env python
import rospy
import struct
from navigation.msg import RawIMUData
from navigation.msg import IMUData
import time

imuoutput=IMUData()

global Ax
global Ay
global Az
global Gx
global Gy
global Gz
global pub



pub = rospy.Publisher('/mis/imu_data', IMUData)




accumulatedAx=0
accumulatedGz=0
totaltime=0
def callback(data):
	global totaltime
	global accumulatedGz

	global accumulatedAx
	global elapsed
	
	
	t=time.time()
	

        #imuoutput.Ax=0.00025*((struct.unpack("b",data.data_buff[4])[0])*256+(struct.unpack("b",data.data_buff[5])[0]))
        #imuoutput.Ay=0.00025*((struct.unpack("b",data.data_buff[6])[0])*256+(struct.unpack("b",data.data_buff[7])[0]))
        #imuoutput.Az=0.00025*((struct.unpack("b",data.data_buff[8])[0])*256+(struct.unpack("b",data.data_buff[9])[0]))
#Taking the two's complement into account: 
	if ((struct.unpack("B",data.data_buff[4])[0])*256+(struct.unpack("B",data.data_buff[5])[0]))>20000:
		imuoutput.Ax=0.00025*(((struct.unpack("B",data.data_buff[4])[0])*256+(struct.unpack("B",data.data_buff[5])[0]))-65536)
	else:
		imuoutput.Ax=0.00025*((struct.unpack("B",data.data_buff[4])[0])*256+(struct.unpack("B",data.data_buff[5])[0]))

	if ((struct.unpack("B",data.data_buff[6])[0])*256+(struct.unpack("B",data.data_buff[7])[0]))>20000:
		imuoutput.Ay=0.00025*(((struct.unpack("B",data.data_buff[6])[0])*256+(struct.unpack("B",data.data_buff[7])[0]))-65536)
	else:
		imuoutput.Ay=0.00025*((struct.unpack("B",data.data_buff[6])[0])*256+(struct.unpack("B",data.data_buff[7])[0]))

	if ((struct.unpack("B",data.data_buff[8])[0])*256+(struct.unpack("B",data.data_buff[9])[0]))>20000:
		imuoutput.Az=0.00025*(((struct.unpack("B",data.data_buff[8])[0])*256+(struct.unpack("B",data.data_buff[9])[0]))-65536)
	else:
		imuoutput.Az=0.00025*((struct.unpack("B",data.data_buff[8])[0])*256+(struct.unpack("B",data.data_buff[9])[0]))
        
	        
	if ((struct.unpack("B",data.data_buff[10])[0])*256+(struct.unpack("B",data.data_buff[11])[0]))>22500:
		imuoutput.Gx=0.02*(((struct.unpack("B",data.data_buff[10])[0])*256+(struct.unpack("B",data.data_buff[11])[0]))-65536)
	else:
		imuoutput.Gx=0.02*((struct.unpack("B",data.data_buff[10])[0])*256+(struct.unpack("B",data.data_buff[11])[0]))

	if ((struct.unpack("B",data.data_buff[12])[0])*256+(struct.unpack("B",data.data_buff[13])[0]))>22500:
		imuoutput.Gy=0.02*(((struct.unpack("B",data.data_buff[12])[0])*256+(struct.unpack("B",data.data_buff[13])[0]))-65536)
	else:
		imuoutput.Gy=0.02*((struct.unpack("B",data.data_buff[12])[0])*256+(struct.unpack("B",data.data_buff[13])[0]))

	if ((struct.unpack("B",data.data_buff[14])[0])*256+(struct.unpack("B",data.data_buff[15])[0]))>22500:
		imuoutput.Gz=0.02*(((struct.unpack("B",data.data_buff[14])[0])*256+(struct.unpack("B",data.data_buff[15])[0]))-65536)
	else:
		imuoutput.Gz=0.02*((struct.unpack("B",data.data_buff[14])[0])*256+(struct.unpack("B",data.data_buff[15])[0]))
        

		
	
        pub.publish(imuoutput)
        #rospy.loginfo(imuoutput) 

	elapsed=time.time()-t
	
	totaltime=totaltime+elapsed
	#print(totaltime)


	#accumulatedAx=accumulatedAx+elapsed*imuoutput.Ax
	#accumulatedGz=accumulatedGz+elapsed*imuoutput.Gz
	#rospy.loginfo(accumulatedAx)
	#rospy.loginfo(accumulatedGz)
	
	

	
	
	

def subscriber_publisher():
	rospy.init_node('nav_subToIMUdata_node')
	
	rospy.Subscriber("/mis/raw_imu_data",RawIMUData,callback)
	rospy.loginfo("nav_readPubIMUdata_node running...")
	
	rospy.spin()
	
        

if __name__=='__main__':

	subscriber_publisher()
        #publisher()
