#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.util.logger import Logger
from src.auto.control import Control

LOGGER = Logger()

# Set up car controls
base_speed = 255
car_control = Control(base_speed)

"""
Abstract:
Unit tests for movement functions in control.py.
"""

LOGGER.info("Testing Brakes")
car_control.stop()
