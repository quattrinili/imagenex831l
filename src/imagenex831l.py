#!/usr/bin/env python

"""
  Author: Alberto Quattrini Li
  Affiliation: AFRL - University of South Carolina
  Date: 06/20/2016

  Description:
  Class for interacting with Imagenex 831l.
"""

import socket # For interacting with the sensor with TCP/IP.
import struct # Preparing the packets to be sent.

import numpy

# MACROS
NUM_BITS_IN_BYTE = 8

# Sonar default values.
SONAR_IP_ADDRESS = "192.168.0.5"
SONAR_PORT = 4040
SONAR_RANGE = 6 # byte 3 (Key ID for value).
SONAR_STEP_DIRECTION = 0 # byte 5 (Key ID for value).
SONAR_START_GAIN = 1 # byte 8 (ID for value).
SONAR_ABSORPTION = 10 # byte 10 (ID for value).
SONAR_STEP_SIZE = 1 # byte 13 (ID for value).
SONAR_PULSE = 1 # byte 14 (ID for value).
SONAR_MIN_RANGE = 1 # byte 15 (ID for value).
SONAR_PITCH_ROLL_MODE = 0 # byte 21 (ID for value).
SONAR_PROFILE_MODE = 0 # byte 23 (ID for value).
SONAR_MOTOR_MODE = 0 # byte 24 (ID for value).
SONAR_FREQUENCY = 20 # byte 25 (ID for the value).


# Values for switch data command (see 831l documentation).
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
BYTE_13 = [0, 3] # Step size. 0: no step. 1: 0.9 degrees/step.
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
        self.current_train_angle = 0 # byte 11.
        self.current_sector_width = 0 # byte 12.
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
                BYTE_3[self.sonar_range],
                BYTE_4,
                BYTE_5[self.step_direction],
                BYTE_6,
                BYTE_7,
                BYTE_8[self.start_gain],
                BYTE_9,
                BYTE_10[self.absorption],
                BYTE_11[self.current_train_angle],
                BYTE_12[self.current_sector_width],
                BYTE_13[self.step_size],
                BYTE_14[self.pulse],
                BYTE_15[self.min_range],
                BYTE_16,
                BYTE_17,
                BYTE_18,
                BYTE_19,
                BYTE_20,
                BYTE_21[self.pitch_roll],
                BYTE_22[self.profile],
                BYTE_23[self.motor],
                BYTE_24,
                BYTE_25[self.frequency],
                BYTE_26
                )
            self.connection.send(request)

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

    def interpret_data(self, data):
        """Interpret raw data.
        """
        # TODO(aql) add status, byte 4.
        range_error_flag = bool(data[4] & 0x01)
        frequency_error_flag = bool(data[4] & 0x02)
        sensor_error_flag = bool(data[4] & 0x04)
        switches_accepted_flag = bool(data[4] & 0x80)
        print "range_error_flag ", range_error_flag
        print "frequency_error_flag ", frequency_error_flag
        print "sensor_error_flag ", sensor_error_flag
        print "switches_accepted_flag ", switches_accepted_flag

        # Processing of bytes 5-6: head position; value in degrees (-180, 180)
        head_high_byte = (data[6] & 0x3E)>>1
        head_low_byte = ((data[6] & 0x01)<<7) | (data[5] & 0x7F)
        head_position = (head_high_byte<<8) | head_low_byte 
        angle = 0.3 * (head_position - 600)
        # step direction: 0 = counter-clockwise, 1 = clockwise.
        step_direction = (data[6] & 0x40)>>6
        print "angle ", (angle, step_direction)

        # byte 7
        for max_range, range_id in BYTE_3.iteritems():
            if range_id == data[7]:
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
        # TODO interpret as 0.025 base, 14 bit two complement.
        print "roll_angle ", roll_angle

        # Processing of bytes 18-19: pitch angle.
        pitch_angle = ((data[19] &0x3F) <<8) | data[18]
        # TODO interpret as 0.025 base, 14 bit two complement.
        print "pitch_angle ", pitch_angle

        # Processing of bytes 20-21: roll acceleration.
        roll_acceleration = ((data[21] &0x3F) <<8) | data[20]
        # TODO interpret as 0.24414 base, 14 bit two complement.
        print "roll_accel ", roll_acceleration

        # Processing of bytes 22-23: pitch acceleration.
        # TODO interpret as 0.24414 base, 14 bit two complement.
        pitch_acceleration = ((data[23] &0x3F) <<8) | data[22]
        print "pitch_accel ", pitch_acceleration

        # Processing of echo data. # TODO(aql) IMX
        ranges = data[32:len(data)-1]  
        print "ranges ", ranges

        return ranges

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

    def close_connection(self):
        if self.connection != None:
            self.connection.close()
            
