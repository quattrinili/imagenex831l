#!/usr/bin/env python

"""
  Author: Alberto Quattrini Li
  Affiliation: AFRL - University of South Carolina
  Date: 06/20/2016

  Description:
  Publishes sensor reading from the imagenex 831l.
"""

import signal # To control Ctrl+C.
import sys # exit.

# Drivers.
import imagenex831l

import rospy
from sensor_msgs.msg import Range
from imagenex_831l.msg import RawRange

# Strings for topics.
SENSOR_NAME = 'imagenex_831l'
TOPIC_SEPARATOR = '/'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'


# Parameters for node.
POLL_FREQUENCY = 10 # Frequency for polling data. Note that the lower bound
                    # depends on the actual sensor frequency.
# END MACROS.

def publish_data_from_depth_sensor():
    """Code that publishes the data from the depth sensor.
    """

    
    # Instantiating publishers.
    range_pub = rospy.Publisher(
        TOPIC_SEPARATOR.join([SENSOR_NAME, SONAR_TOPIC_NAME]),
        Range, queue_size=10)
    range_raw_pub = rospy.Publisher(
        TOPIC_SEPARATOR.join([SENSOR_NAME, SONAR_RAW_TOPIC_NAME]),
        RawRange, queue_size=10)

    # Start the node.
    rospy.init_node('imagenex_831l', anonymous=True)

    # Get parameters.
    frequency = rospy.get_param('poll_frequency', POLL_FREQUENCY) # Frequency to read sensor.


    # Initialize sensor.
    sensor = imagenex_831l.Imagenex831L()

    node = rospy.Rate(frequency)

    # Loop.
    while not rospy.is_shutdown():
        # Creation of the messages.
        sonar_msg = Range()
        sonar_raw_msg = RawRange()
        try:
            # Reading of the sensor.
            sensor.send_request()
            raw_data = sensor.read_data()

            # Populating the messages with the data.
            time = rospy.get_rostime()

            sonar_raw_msg.header.stamp = time
            sonar_raw_msg.header.frame_id = "sonar"
            sonar_raw_msg.data = raw_data
            # For logging purpose.
            rospy.logdebug(sonar_raw_msg)

            sonar_msg.header.stamp = time
            sonar_msg.header.frame_id = "sonar"

            # Publish ROS messages.
            range_raw_pub.publish(sonar_raw_msg)
            range_pub.publish(sonar_msg)

        except:
            # Error when reading data.
            rospy.logerr("Exception when reading depth data.")

        # Keep the frequency.
        node.sleep()
        node.spinOnce()

    # Terminate sensor.
    sensor.close_connection()


if __name__ == '__main__':
    publish_data_from_sonar_sensor()
