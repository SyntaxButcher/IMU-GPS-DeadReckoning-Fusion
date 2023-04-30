#!/usr/bin/env python
# -*- coding: utf-8 -*-S

import rospy
import serial
from lab4.msg import IMUmsg
import numpy as np 
 
def RPY2Quaternion(roll, pitch, yaw):
  roll = roll * pi / 180
  pitch = pitch * pi / 180
  yaw = yaw * pi / 180
  QuatX = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  QuatY = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  QuatZ = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  QuatW = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [QuatX, QuatY, QuatZ, QuatW]

def Spinner():
	SENSOR_NAME = "IMU"
	pub = rospy.Publisher('IMU', custom, queue_size=10)
	rospy.init_node('IMyou_node', anonymous=True)
	serial_port = rospy.get_param('~port','/dev/ttyUSB0')
	serial_baud = rospy.get_param('~baudrate',115200)
	port = serial.Serial(serial_port, serial_baud, timeout=3.)
	rate = rospy.Rate(40)
	IMU = custom()
	IMU.header.frame_id = "IMU"
	IMU.header.seq = 0
	IMU.data.header.frame_id = "IMU/data"
	IMU.data.header.seq = 0
	IMU.mag.header.frame_id = "IMU/mag"
	IMU.mag.header.seq = 0
	
	while not rospy.is_shutdown():
		line = port.readline()
		line = str(line)
		if "VNYMR" in line:
			line_list = line.split(',')
			IMU.mag.magnetic_field.x, IMU.mag.magnetic_field.y, IMU.mag.magnetic_field.z = float(line_list[4].replace('\\x00','')), float(line_list[5].replace('\\x00','')), float(line_list[6].replace('\\x00',''))
			IMU.data.linear_acceleration.x, IMU.data.linear_acceleration.y, IMU.data.linear_acceleration.z = float(line_list[7].replace('\\x00','')), float(line_list[8].replace('\\x00','')), float(line_list[9].replace('\\x00',''))
			IMU.data.angular_velocity.x, IMU.data.angular_velocity.y, IMU.data.angular_velocity.z = float(line_list[10].replace('\\x00','')), float(line_list[11].replace('\\x00','')), float(((line_list[12])[:-8]).replace('\\x00',''))
			IMU.Yaw, IMU.Pitch, IMU.Roll = float(line_list[1].replace('\\x00','')), float(line_list[2].replace('\\x00','')), float(line_list[3].replace('\\x00',''))
			IMU.data.orientation.x, IMU.data.orientation.y, IMU.data.orientation.z, IMU.data.orientation.w = RPY2Quaternion(IMU.Roll, IMU.Pitch, IMU.Yaw)
			IMU.header.stamp = rospy.Time.now()
			IMU.data.header.stamp = rospy.Time.now()
			IMU.mag.header.stamp = rospy.Time.now()
			rospy.loginfo(IMU)
			pub.publish(IMU)
			IMU.header.seq += 1
			IMU.data.header.seq += 1
			IMU.mag.header.seq += 1

		
	
if __name__ == '__main__':
	try:
		Spinner()
	except rospy.ROSInterruptException:
		pass
