#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from src.util.logger import Logger
from src.auto.control import Control

LOGGER = Logger()

# Set up car controls
base_speed = 256
car = Control(base_speed)

"""
Abstract:
Command line test for movement.
"""

motor_vals = [ float(m) for m in sys.argv[1:5] ]

car.move(motor_vals)
time.sleep(int(sys.argv[5]) if len(sys.argv) > 5 else 2)
car.stop()
