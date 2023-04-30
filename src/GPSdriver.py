#!/usr/bin/env python
# -*- coding: utf-8 -*-S

import rospy
import serial
import utm
from math import sin, pi
from lab4.msg import GPSmsg

def parse_GPGGA(line):
	line_array = line.split(',')
	if line_array[3] == "N":
		GPS_lat = float(line_array[2])/100
	else:
		GPS_lat = float(line_array[2])/-100
	if line_array[5] == "E":	
		GPS_lon = float(line_array[4])/100
	else:
		GPS_lon = float(line_array[4])/-100
	GPS_alt = line_array[9]
	return GPS_lat, GPS_lon, GPS_alt

def talker():
	SENSOR_NAME = "GPS"
	pub = rospy.Publisher('GPS', custom, queue_size=10)
	rospy.init_node('GPS_node', anonymous=True)
	serial_port = rospy.get_param('~port','/dev/ttyUSB0')
	serial_baud = rospy.get_param('~baudrate',4800)
	port = serial.Serial(serial_port, serial_baud, timeout=3.)
	rate = rospy.Rate(10)
	GPS = custom()
	while not rospy.is_shutdown():
		rospy.sleep(0.2)
		line = port.readline()
		line = str(line)
		if "GPGGA" in line:
			GPS.latitude, GPS.longitude, GPS.altitude = parse_GPGGA(line)
			GPS.utm_easting, GPS.utm_northing, GPS.Zone, GPS.letter = utm.from_latlon(GPS.latitude, GPS.longitude)
			rospy.loginfo(GPS)
			pub.publish(GPS)


			
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

  
    

