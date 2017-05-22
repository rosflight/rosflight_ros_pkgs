################################################################################
#
# Copyright (c) 2017 Taylor Pool, BYU MAGICC Lab.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

# Taylor Pool
# December 1, 2016
#AUVSI Project

# This script contains a function that converts given GPS coordinates into waypoints for ROS.
#The input supports two types of GPS coordinates
#1) Long Degree Decimal Format
#	ex: -34.6830975
#2) Degrees-Minutes-Seconds Format
#	ex: N78-45-76.23
#The function to be called for conversion is to_meters(originLat, originLong, originAlt, newLat, newLong, newAlt, flag)
#The two formats listed above can be used simultaeously in the same function call.
#NOTE: Degrees-Minutes-Seconds Format is the only format that uses NSEW directions in front.
#NOTE: Degrees-Minutes-Seconds Format needs to be surrounded with quotations when passed as an argument.

from math import pi
from math import cos
from math import sqrt

#constant
EARTH_RADIUS = 6371008.0							#in meters

#Function decimal_degrees
#Pre: string is in format [N/E or S/W]DD-MM-SS.SS
#Post: Returns GPS component in long decimal format
def decimal_degrees (string):
	a = 0

	firstLetter = string[0]
	if firstLetter == 'N' or firstLetter == 'E':
		a = 1
	elif firstLetter == 'S' or firstLetter == 'W':
		a = -1

	lessString = string.strip("NSEW ")

	values = lessString.split('-', 2)

	d = float(values[0])
	m = float(values[1])
	s = float(values[2])

	decimal = a*(d+(m/60.0)+(s/3600.0))

	return decimal


#Function meter_convert
#Takes in long decimal GPS format and outputs the destination in meters
#Pre: Lat and Long are given in decimal degrees, and altitude is in meters
#Post: Returns destination in tuple format [latChange, longChange, newAlt, distance]
def meter_convert(originLat, originLong, originAlt, newLat, newLong, newAlt):

	#Find CrossSectional Radius of Earth for Given Latitude Degree
	crossRadius = cos(originLat*pi/180.0)*EARTH_RADIUS

	GPSdestination = [newLat, newLong, -newAlt]	#Given in terms of N, E, D

	#Convert Change in GPS Coordinates to Change in Meters
	latChange = ((newLat - originLat)*pi/180.0)*EARTH_RADIUS		#in meters

	longChange = ((newLong - originLong)*pi/180.0)*crossRadius

	altitudeChange = newAlt - originAlt

	#Compute Total Distance to Fly
	distance = sqrt(pow(latChange, 2.0) + pow(longChange, 2.0) + pow(altitudeChange, 2.0))

	#New Waypoint Given in Terms of [North, East, Down]. In meters.
	destination = [latChange, longChange, newAlt, distance]

	return destination


#Function to_meters
#Pre: input of all the data needed to fly to a waypoint
#flag: tells whether altitude is in meters or feet
#		0: meters		else: feet
#Post: outputs destination given in meters that ROS can understand.
def to_meters(originLat, originLong, originAlt, newLat, newLong, newAlt, flag):

	#Altitude in Feet needs to be converted to Meters
	if (flag != 0):
		originAlt = .305*originAlt
		newAlt = .305*newAlt

	GPSorigin = [originLat, originLong, originAlt]
	GPSdestination = [newLat, newLong, newAlt]

	values = [str(originLat), str(originLong), str(newLat), str(newLong)]
	newValues = []

	for value in values:
		if ("N" in value) or ("S" in value) or ("E" in value) or ("W" in value) == 1:
			#print "Degrees Minutes Seconds Format"
			#print value
			newValues.append(decimal_degrees(value))
			#print decimal_degrees(value)
		else:
			#print "Long Decimal Format"
			newValues.append(float(value))

	destination = meter_convert(newValues[0], newValues[1], originAlt, newValues[2], newValues[3], newAlt)

	#Test Output
	#print("Origin in GPS Coordinates: " + str(GPSorigin))
	#print("\n")
	#print("Destination in GPS Coordinates: " + str(GPSdestination))
	#print("\n")
	#print("Destination Coordinates (Meters) with Distance: " + str(destination))

	return destination



#######################################################################################################


#Test
#test = to_meters("N90-90-76.45", "W45-67-23.54", 20.0, -40.257049176511316, 111.65421836078167, 20.0, 0)

#if __name__ == __main__:
#	toMeters(40.25787274333326, -111.65480308234692, -20.0, 40.257049176511316, -111.65421836078167, -20.0, 0)


#######################################################################################################################

#Sample Output:
#tpool2@FB280-09:/media/tpool2/TAYLORPOOL/AUVSI/rosflight_utils/src$ python WaypointConvert.py
#Degrees Minutes Seconds Format
#N90-90-76.45
#91.5212361111
#Degrees Minutes Seconds Format
#W45-67-23.54
#-46.1232055556
#Long Decimal Format
#Long Decimal Format
#Origin in GPS Coordinates: ['N90-90-76.45', 'W45-67-23.54', 20.0]


#Destination in GPS Coordinates: [-40.257049176511316, 111.65421836078167, 20.0]


#Destination Coordinates (Meters) with Distance: [-14653095.165621338, -465750.5181201244, 20.0, 14660495.267141715]
