#!/usr/bin/env python

"""
  Author: Alberto Quattrini Li
  Affiliation: AFRL - University of South Carolina
  Date: 12/20/2016

  Description:
  Publishes sensor reading from the imagenex 831l.
"""

# Drivers.
from imagenex831l.imagenex831l_driver import Imagenex831L

import rospy

from dynamic_reconfigure.server import Server
from imagenex831l.cfg import Imagenex831LConfig

from imagenex831l.msg import ProcessedRange
from imagenex831l.msg import RawRange

# Strings for topics.
SENSOR_NAME = 'imagenex831l'
TOPIC_SEPARATOR = '/'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'


# Parameters for node.
POLL_FREQUENCY = 1000 # Frequency for polling data. Note that the lower bound
                    # depends on the actual sensor frequency. TODO frequency depending on range.
RESET_TIMEOUT = 1 # Timeout (s) to reset node, if no message has been produced.
# END MACROS.

class SonarNode(object):
    def __init__(self):
        """Initialization of the node.
        """
        # Start the node.
        rospy.init_node('imagenex831l')

        # Instantiating publishers.
        self.range_pub = rospy.Publisher(
            TOPIC_SEPARATOR.join([SENSOR_NAME, SONAR_TOPIC_NAME]),
            ProcessedRange, queue_size=10)
        self.range_raw_pub = rospy.Publisher(
            TOPIC_SEPARATOR.join([SENSOR_NAME, SONAR_RAW_TOPIC_NAME]),
            RawRange, queue_size=10)

        # Get parameters.
        self.frequency = rospy.get_param('poll_frequency', POLL_FREQUENCY) # Frequency to read sensor.

        # Initialize sensor.
        self.sensor = Imagenex831L()

        # Initialize parameter server.
        self.parameter_server = Server(Imagenex831LConfig, self.parameters_callback)

        self.first_exception_time = None
    
    def parameters_callback(self, config, level):
        """Set parameters of the node for dynamic_reconfigure.
        """
        config = self.sensor.set_parameters(config)
        return config

    def spin(self):
        """Code that publishes the data from the depth sensor.
        """
        node = rospy.Rate(self.frequency)

        # Loop.
        while not rospy.is_shutdown():
            # Creation of the messages.
            
            sonar_msg = ProcessedRange()
            sonar_raw_msg = RawRange()
            current_time = rospy.get_rostime()
            try:
                # Reading of the sensor.
                self.sensor.send_request()
                raw_data = self.sensor.read_data()

                # Populating the messages with the data.
                

                sonar_raw_msg.header.stamp = current_time
                sonar_raw_msg.header.frame_id = "sonar"
                sonar_raw_msg.data = raw_data
                # For logging purpose.
                rospy.logdebug(sonar_raw_msg)

                sonar_msg.header.stamp = current_time
                sonar_msg.header.frame_id = "sonar"
                self.sensor.interpret_data(raw_data, sonar_msg)

                # Publish ROS messages.
                self.range_raw_pub.publish(sonar_raw_msg)
                self.range_pub.publish(sonar_msg)
                if self.first_exception_time:
                    self.first_exception_time = None
            except:
                # Error when reading data.
                if self.first_exception_time is None:
                    rospy.logerr("Exception when reading sonar data.")
                    self.first_exception_time = current_time
                else:
                    if current_time - self.first_exception_time > rospy.Duration(RESET_TIMEOUT):
                        rospy.signal_shutdown("Sonar sensor not ready")

            # Keep the frequency.
            node.sleep()

        # Terminate sensor.
        self.sensor.close_connection()


if __name__ == '__main__':
    sonar_node = SonarNode()
    sonar_node.spin()
