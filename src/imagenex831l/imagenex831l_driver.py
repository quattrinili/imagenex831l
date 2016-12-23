#!/usr/bin/env python

"""
  Author: Alberto Quattrini Li
  Affiliation: AFRL - University of South Carolina
  Date: 12/22/2016

  Description:
  Class for interacting with Imagenex 831l.

  TODO Clean the code from macros that are not useful.
  TODO Make consistent the way values are passed to the message.
"""

import socket # For interacting with the sensor with TCP/IP.
import struct # Preparing the packets to be sent.

import numpy

from utilities import *

# MACROS
NUM_BITS_IN_BYTE = 8

# Sonar default values.
SONAR_IP_ADDRESS = "192.168.0.5"
SONAR_PORT = 4040
SONAR_RANGE = 10 # byte 3 (Key ID for value).
SONAR_STEP_DIRECTION = 0 # byte 5 (Key ID for value).
SONAR_START_GAIN = 10 # byte 8 (ID for value).
SONAR_ABSORPTION = 10 # byte 10 (ID for value).
SONAR_STEP_SIZE = 3 # byte 13 (ID for value).
SONAR_PULSE = 10 # byte 14 (ID for value).
SONAR_MIN_RANGE = 1 # byte 15 (ID for value).
SONAR_PITCH_ROLL_MODE = 0 # byte 21 (ID for value).
SONAR_PROFILE_MODE = 0 # byte 23 (ID for value).
SONAR_MOTOR_MODE = 0 # byte 24 (ID for value).
SONAR_FREQUENCY = 20 # byte 25 (ID for the value).
ALLOWED_FREQUENCIES = range(2150,2350+1, 5)

# Values for switch data command (see 831L documentation).
NUM_BYTES = 27 # Number of bytes for request.
RESERVED_VALUE = 0x00 # Value for reserved bytes.
BYTE_0 = 0xfe # Switch data header byte 0.
BYTE_1 = 0x44 # Switch data header byte 1.
BYTE_2 = RESERVED_VALUE
BYTE_3 = {0.125: 2, 0.25: 4, 0.5: 6, 0.75: 8, # key: range in m. 
    1: 10, 2: 20, 3: 30, 4: 40, 5: 50, 6: 60} # value: decimal for byte 3.
BYTE_4 = RESERVED_VALUE
BYTE_5 = {0: 0x00, 1: 64} # 0: normal operation, 1: reverse step direction. TODO check if it's the correct order.
BYTE_6 = RESERVED_VALUE
BYTE_7 = RESERVED_VALUE
BYTE_8 = range(0, 40+1) # Start gain: MIN, MAX, 1dB increments.
BYTE_9 = RESERVED_VALUE
BYTE_10 = range(0, 255+1) # absorption_in_dB_per_m * 100 (MIN, MAX). NOTE: AVOID 253.
BYTE_11 = [0, 30, 60, 90, 120] # train_angle.
BYTE_12 = [0, 30, 60, 90, 120] # sector width.
#BYTE_13 = [0, 3] # Step size. 0: no step. 1: 0.9 degrees/step.
BYTE_14 = range(1, 100+1) # pulse length in microseconds.
BYTE_15 = range(0, 250+1) # min range in meters; 0 to 2.5m.
BYTE_16 = RESERVED_VALUE
BYTE_17 = RESERVED_VALUE
BYTE_18 = RESERVED_VALUE
BYTE_19 = 25 # Data points.
BYTE_20 = 8 # Data bits.
BYTE_21 = {0: 1, 1: 255} # 0: Interrogate PR sensor, 1: Calibrate pitch/roll.
BYTE_22 = [0, 1] # 0: Profile off, 1: profile on.
BYTE_23 = [0, 1] # 0: normal operation, 1: Calibrate sonar head transducer.
BYTE_24 = RESERVED_VALUE
BYTE_25 = range(80, 120+1) # 2.15 MHz to 2.35 MHz.
BYTE_26 = 0xFD # Termination byte.

# Values for interpreting response (see 831L documentation).
ANGLE_PER_BIT = 0.025
ACCELERATION_PER_BIT = 0.24414
TWO_COMPLEMENTS_NUM_BITS = 14

# Strings for topics.
SENSOR_NAME = '831l'
TOPIC_SEPARATOR = '/'
SONAR_TOPIC_NAME = 'sonar'



# Parameters for node.
POLL_FREQUENCY = 10 # Frequency for polling data. Note that the lower bound
                    # depends on the actual sensor frequency.
# END MACROS.

class Imagenex831L():
    def __init__(self, ip_address=SONAR_IP_ADDRESS, port=SONAR_PORT,
        sonar_range=SONAR_RANGE, step_direction=SONAR_STEP_DIRECTION,
        start_gain=SONAR_START_GAIN, absorption=SONAR_ABSORPTION,
        step_size=SONAR_STEP_SIZE, pulse=SONAR_PULSE,
        min_range=SONAR_MIN_RANGE, frequency=SONAR_FREQUENCY):
        """Class constructor for the sonar. TODO(aql) more complete doc.
        """
        # TCP/IP connection.
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.connection.connect((ip_address, port))
        except socket.error, msg:
            self.connection = None

        # Setting parameters.
        self.request_format = str(NUM_BYTES) + "B"
        self.sonar_range = sonar_range # byte 3.
        self.step_direction = step_direction # byte 5.
        self.start_gain = start_gain # byte 8.
        self.absorption = absorption # byte 10.
        self.train_angle = 120 # byte 11.
        self.sector_width = 120 # byte 12.
        self.step_size = step_size # byte 13.
        self.pulse = pulse # byte 14.
        self.min_range = min_range # byte 15.
        self.pitch_roll = SONAR_PITCH_ROLL_MODE # byte 21.
        self.profile = SONAR_PROFILE_MODE # byte 22.
        self.motor = SONAR_MOTOR_MODE # byte 23.
        self.frequency = frequency # byte 25.
        self.response_size = NUM_BYTES * 8 # number of bytes in the response.


    def send_request(self):
        """Send request of data through TCP/IP. TODO(aql) more complete doc.
        """
        if self.connection == None:
            print "No Connection"
        if self.connection != None:
            request = struct.pack(self.request_format, 
                BYTE_0,
                BYTE_1,
                BYTE_2,
                #BYTE_3[self.sonar_range],
                self.sonar_range,
                BYTE_4,
                BYTE_5[self.step_direction],
                BYTE_6,
                BYTE_7,
                #BYTE_8[self.start_gain],
                self.start_gain,
                BYTE_9,
                #BYTE_10[self.absorption],
                self.absorption,
                #BYTE_11[self.train_angle],
                self.train_angle,
                #BYTE_12[self.sector_width],
                self.sector_width,
                #BYTE_13[self.step_size],
                self.step_size,
                #BYTE_14[self.pulse],
                self.pulse,
                #BYTE_15[self.min_range],
                self.min_range,
                BYTE_16,
                BYTE_17,
                BYTE_18,
                BYTE_19,
                BYTE_20,
                BYTE_21[self.pitch_roll],
                BYTE_22[self.profile],
                BYTE_23[self.motor],
                BYTE_24,
                self.frequency,
                BYTE_26
                )
            self.connection.send(request)
            if self.step_direction == 1:
                self.step_direction = 0

    def read_data(self):
        """Receive request of data through TCP/IP. TODO(aql) more complete doc.
        """
        total_num_bytes = 33 # Number of bytes common to every package.

        # If Profile is ON then N number of data.
        if self.profile == 0:
            data_num_bytes = 250
            total_num_bytes += data_num_bytes

        # Number of bytes received and data format.
        bits_received = total_num_bytes * NUM_BITS_IN_BYTE
        data_format = str(total_num_bytes) + "B"

        data = None
        if self.connection != None:
            # Receiving data.
            data = struct.unpack(data_format,
                self.connection.recv(bits_received))

        return data

    def interpret_data(self, data, message=None):
        """Interpret raw data. TODO(aql) more complete doc.


        Args:
            data(bytes): raw data from sensor.
            message(ProcessedRange): if not None, it saves the message fields.
        """
        # Processing of byte 4: status.
        range_error_flag = bool(data[4] & 0x01)
        frequency_error_flag = bool(data[4] & 0x02)
        internal_error_flag = bool(data[4] & 0x04)
        switches_accepted_flag = bool(data[4] & 0x80)
        print "range_error_flag ", range_error_flag
        print "frequency_error_flag ", frequency_error_flag
        print "sensor_error_flag ", internal_error_flag
        print "switches_accepted_flag ", switches_accepted_flag

        # Processing of bytes 5-6: head position; value in degrees (-180, 180)
        head_high_byte = (data[6] & 0x3E)>>1
        head_low_byte = ((data[6] & 0x01)<<7) | (data[5] & 0x7F)
        head_position = (head_high_byte<<8) | head_low_byte 
        head_position = 0.3 * (head_position - 600)
        # step direction: 0 = counter-clockwise, 1 = clockwise.
        step_direction = (data[6] & 0x40)>>6
        print "head_position ", (head_position, step_direction)

        # byte 7
        maximum_range = 0
        for max_range, range_id in BYTE_3.iteritems():
            if range_id == data[7]:
                maximum_range = max_range
                print "max range ", max_range

        # Processing of bytes 8-9: profile range; in centimetres.
        profile_range_high_byte = (data[9] & 0x7E)>>1
        profile_range_low_byte = ((data[9] & 0x01)<<7) | (data[8] & 0x7F)        
        profile_range = (profile_range_high_byte<<8) | profile_range_low_byte
        profile_range /= 100.0
        print "profile range ", profile_range

        # Processing of bytes 10-11: number of Echo Data Bytes returned.
        data_bytes_high_byte = (data[11] & 0x7E)>>1
        data_bytes_low_byte = ((data[11] & 0x01)<<7) | (data[10] & 0x7F)
        data_bytes = (data_bytes_high_byte<<8) | data_bytes_low_byte
        print "data_bytes ", data_bytes

        # Processing of bytes 16-17: roll angle.
        roll_angle = ((data[17] &0x3F) <<8) | data[16]
        roll_angle = twos_complement(roll_angle, TWO_COMPLEMENTS_NUM_BITS)
        roll_angle = convert_to_weighted_bits(bin(roll_angle), ANGLE_PER_BIT, 
            num_bits=TWO_COMPLEMENTS_NUM_BITS,
            least_significant_bit_zero=False)
        roll_angle_error_alarm_flag = data[17] & 0x04
        roll_angle_new_data_flag = data[17] & 0x08
        print "roll_angle ", roll_angle

        # Processing of bytes 18-19: pitch angle.
        pitch_angle = ((data[19] &0x3F) <<8) | data[18]
        pitch_angle = twos_complement(pitch_angle, TWO_COMPLEMENTS_NUM_BITS)
        pitch_angle = convert_to_weighted_bits(bin(pitch_angle), ANGLE_PER_BIT, 
            num_bits=TWO_COMPLEMENTS_NUM_BITS,
            least_significant_bit_zero=False)
        pitch_angle_error_alarm_flag = data[17] & 0x04
        pitch_angle_new_data_flag = data[17] & 0x08
        print "pitch_angle ", pitch_angle

        # Processing of bytes 20-21: roll acceleration.
        roll_acceleration = ((data[21] &0x3F) <<8) | data[20]
        roll_acceleration = twos_complement(roll_acceleration, 
            TWO_COMPLEMENTS_NUM_BITS)
        roll_acceleration = convert_to_weighted_bits(bin(roll_acceleration), 
            ACCELERATION_PER_BIT, 
            num_bits=TWO_COMPLEMENTS_NUM_BITS,
            least_significant_bit_zero=False)
        roll_acceleration_error_alarm_flag = data[17] & 0x04
        roll_acceleration_new_data_flag = data[17] & 0x08
        print "roll_accel ", roll_acceleration

        # Processing of bytes 22-23: pitch acceleration.
        pitch_acceleration = ((data[23] &0x3F) <<8) | data[22]
        pitch_acceleration = twos_complement(pitch_acceleration, 
            TWO_COMPLEMENTS_NUM_BITS)
        pitch_acceleration = convert_to_weighted_bits(bin(pitch_acceleration), 
            ACCELERATION_PER_BIT, 
            num_bits=TWO_COMPLEMENTS_NUM_BITS,
            least_significant_bit_zero=False)
        pitch_acceleration_error_alarm_flag = data[17] & 0x04
        pitch_acceleration_new_data_flag = data[17] & 0x08
        print "pitch_accel ", pitch_acceleration

        # Processing of echo data. # TODO(aql) IMX
        ranges = data[32:len(data)-1]  
        print "ranges ", ranges

        if message:
            message.intensity = ranges
            message.range_error = range_error_flag
            message.frequency_error = frequency_error_flag
            message.internal_error = internal_error_flag
            message.switches_accepted = switches_accepted_flag

            message.head_position = head_position
            message.step_direction = step_direction
            message.max_range = maximum_range
            message.profile_range = profile_range
            message.roll_angle = roll_angle
            message.pitch_angle = pitch_angle
            message.roll_acceleration = roll_acceleration
            message.pitch_acceleration = pitch_acceleration

        """TODO(aql) check what information to save in the range message.
        # REP117 implementation http://www.ros.org/reps/rep-0117.html.
        # TODO(aql) read minimum range in meters.
        if self.min_range/100 <= value and value <= self.maximum_range:
            # This is a valid measurement.
            pass
        elif numpy.isinf(value) and value < 0:
            # Object too close to measure.
        elif numpy.isinf(value) and value > 0:
            # No objects detected in range.
        elif numpy.isnan(value):
            # This is an erroneous, invalid, or missing measurement.
        else:
            # The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
        """

    def set_parameters(self, config):
        """Set parameters. TODO(aql) more complete doc.


        Args:
            config: structure coming from dynamic_reconfigure.
        """
        self.sonar_range = config.max_range # byte 3.
        self.step_direction = config.step_direction # byte 5.
        # Ensure that step_direction returns to 0 as it is one-time bit to
        # reverse the direction of the rotating transducer.
        config.step_direction = 0
        self.start_gain = config.start_gain # byte 8.
        if config.absorption == 253:
            # Avoid problems with the end character of the switch command.
            config.absorption = 252
        self.absorption = config.absorption # byte 10.
        config.train_angle = my_round(config.train_angle, base=3)
        self.train_angle = config.train_angle # byte 11.
        config.sector_width = my_round(config.sector_width, base=3)
        self.sector_width = config.sector_width # byte 12.
        self.step_size = config.step_size # byte 13.
        self.pulse = config.pulse # byte 14.
        self.min_range = config.min_range # byte 15.
        self.pitch_roll = config.pitch_roll_mode # byte 21.
        self.profile = config.profile_mode # byte 22.
        self.motor = config.motor_mode # byte 23.
        config.frequency = my_round(config.frequency)
        frequency_id = ALLOWED_FREQUENCIES.index(config.frequency)
        self.frequency = BYTE_25[frequency_id] # byte 25.
        return config

    def close_connection(self):
        """Close TCP/IP connection.
        """
        if self.connection != None:
            self.connection.close()
