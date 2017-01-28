#!/usr/bin/env python

"""
  Author: Sharmin Rahman
  Affiliation: AFRL - University of South Carolina
  Date: 12/22/2016

  Description:
  Test of sonar.
"""

from imagenex831l.imagenex831l_driver import Imagenex831L

s = Imagenex831L()

# Setting params as windows application.
s.absorption = 171
s.start_gain = 21
s.pulse = 10
s.range = 1
s.current_sector_width = 0
s.current_train_angle = 0

s.send_request()
data = s.read_data()
s.interpret_data(data)

s.close_connection()

