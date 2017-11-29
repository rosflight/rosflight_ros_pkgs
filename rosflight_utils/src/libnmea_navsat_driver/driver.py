# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from rosflight_msgs.msg import GPS

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


class RosNMEADriver(object):
    def __init__(self):
        self.fix_pub = rospy.Publisher('gps', GPS, queue_size=1)
        # self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)

        self.speed = 0
        self.ground_course = 0

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentece was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = GPS()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']
            if data['fix_valid']:
                self.speed = data['speed']
                if not math.isnan(data['true_course']):
                    self.ground_course = data['true_course']

        if not self.use_RMC and 'GGA' in parsed_sentence:
            data = parsed_sentence['GGA']
            gps_qual = data['fix_type']
            if gps_qual > 0:
                current_fix.fix = True
                current_fix.NumSat = data['num_satellites']
            # current_fix.status.service = NavSatStatus.SERVICE_GPS

            current_fix.header.stamp = current_time

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            hdop = data['hdop']
            current_fix.covariance = hdop ** 2
            # current_fix.position_covariance[4] = hdop ** 2
            # current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
            # current_fix.position_covariance_type = \
            #     NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            current_fix.speed = self.speed
            current_fix.ground_course = self.ground_course

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                self.time_ref_pub.publish(current_time_ref)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                current_fix.fix = True
                current_fix.NumSat = data['num_satellites']

                # current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')

                # current_fix.position_covariance_type = \
                #     NavSatFix.COVARIANCE_TYPE_UNKNOWN
                if data['fix_valid']:
                    current_fix.speed = data['speed']
                    if not math.isnan(data['true_course']):
                        current_fix.ground_course = data['true_course']
                    else:
                        current_fix.ground_course = self.ground_course

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                    self.time_ref_pub.publish(current_time_ref)

        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            """Add the TF prefix"""
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
