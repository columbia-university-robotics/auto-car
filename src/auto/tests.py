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
base_speed = 128
car = Control(base_speed)

"""
Abstract:
Unit tests for movement functions in control.py.
"""

LOGGER.info("Testing Brakes")
car.go()
time.sleep(2)
car.stop()
time.sleep(2)
car.crab_l()
time.sleep(4)
car.stop()
time.sleep(2)
car.crab_r()
time.sleep(4)
car.stop()
time.sleep(2)
car.turn_l()
time.sleep(2)
car.stop()
time.sleep(2)
car.turn_r()
time.sleep(2)
car.stop()
time.sleep(2)
car.rot_l()
time.sleep(2)
car.stop()
time.sleep(2)
car.rot_r()
time.sleep(2)
car.stop()
