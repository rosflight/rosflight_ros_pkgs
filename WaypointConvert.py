# Taylor Pool
# November 9, 2016

# This script contains a function that converts given GPS coordinates into waypoints for ROS.

from math import pi
from math import cos
from math import sqrt
from math import pow

#constant
earthRadius = 6371008.0							#in meters

#Set the Origin GPS Location
GPSlat = 40.25697101674908									#in degrees
GPSlong = -111.65421467274427								#in degrees

#longEast = 0								#if 0, degrees are East. West otherwise.
#if longEast != 0:
#	GPSlong = -GPSlong			#Makes west direction negative in keeping with East Orientation

altitudeOrigin = 20.0							#in meters above sea level
GPSorigin = [GPSlat, GPSlong, -altitudeOrigin]		#Given in terms of N, E, D

#Find CrossSectional Radius of Earth for Given Latitude Degree
crossRadius = cos(GPSlat*pi/180.0)*earthRadius

#Input GPS coordinates to fly to
newLat = 40.25794221775606
newLong = -111.65423478931189
newAltitude = 20.0								#in meters above sea level
GPSdestination = [newLat, newLong, -newAltitude]	#Given in terms of N, E, D

#Convert Change in GPS Coordinates to Change in Meters

latChange = ((newLat - GPSlat)*pi/180.0)*earthRadius		#in meters

longChange = ((newLong - GPSlong)*pi/180.0)*crossRadius

altitudeChange = newAltitude - altitudeOrigin

#New Waypoint Given in Terms of [North, East, Down]. In meters.
destination = [latChange, longChange, -newAltitude]

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



		


