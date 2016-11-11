#!/usr/bin/python
# Taylor Pool
# November 9, 2016

# This script contains a function that converts given GPS coordinates into waypoints for ROS.

from math import pi
from math import cos
from math import sqrt
from math import pow

#constant
earthRadius = 6371008.0							#in meters

def toMeters(originLat, originLong, originAlt, newLat, newLong, newAlt, flag):

	#Flag Key
	#0: GPS Coordinates are given in long decimal format, altitude is given in meters
	#1: GPS Coordinates are given in DirectionDegreesMinutesSeconds, altitude is given in feet

	GPSorigin = [originLat, originLong, originAlt]

	if (flag == 0):

		#Find CrossSectional Radius of Earth for Given Latitude Degree
		crossRadius = cos(originLat*pi/180.0)*earthRadius
		
		GPSdestination = [newLat, newLong, -newAlt]	#Given in terms of N, E, D

		#Convert Change in GPS Coordinates to Change in Meters

		latChange = ((newLat - originLat)*pi/180.0)*earthRadius		#in meters

		longChange = ((newLong - originLong)*pi/180.0)*crossRadius

		altitudeChange = newAlt - originAlt

		#New Waypoint Given in Terms of [North, East, Down]. In meters.
		destination = [latChange, longChange, newAlt]

	if (flag == 1):
		print ("not done")

	#Compute Total Distance to Fly

	distance = sqrt(pow(latChange, 2.0) + pow(longChange, 2.0) + pow(altitudeChange, 2.0))

	#Test Output
	print("Origin in GPS Coordinates: " + str(GPSorigin))
	print("\n")
	print("Destination in GPS Coordinates: " + str(GPSdestination))
	print("\n")
	print("Distance from Origin to Destination (Meters): " + str(distance))
	print("\n")
	print("Destination Coordinates (Meters): " + str(destination))

	return destination

#Test
test = toMeters(40.25787274333326, -111.65480308234692, -20.0, 40.257049176511316, -111.65421836078167, -20.0, 0)

#if __name__ == __main__:
#	toMeters(40.25787274333326, -111.65480308234692, -20.0, 40.257049176511316, -111.65421836078167, -20.0, 0)




		


