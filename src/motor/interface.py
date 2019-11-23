#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import os
import sys
from enum import Enum
import atexit
import rospy
import numpy as np
import time
import threading
from os.path import join, dirname
from std_msgs.msg import String

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(join(dirname(__file__), '../..'))
from src.util.logger import Logger
import Adafruit_GPIO.I2C as I2C
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class Interface:

    """
    Abstract:
    Simple script to baseline motor execution.
    """

    def __init__(self):
        self.LOGGER = Logger(self)
        self.controller = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
        self.motors = [0, 0, 0, 0]
        # Initialize motor subscribers
        rospy.init_node('interface')
        rospy.Subscriber("/motor", String, self.on_motor_callback, queue_size=1)
        # Turn off motors when the script exits.
        atexit.register(self.turn_off_motors)

        # motor tracking
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.LOGGER.info(str(self.motors))
            # if self.motors[0] is self.motors[2] and self.motors[1] is self.motors[3]:
            #     self.send([0, 0, 0, 0])
            # else:
            #     am = np.argmax(self.motors)
            #     self.LOGGER.info("AM " + am)
            #     if am == 0:
            #         self.send([self.speed, self.speed, self.speed, self.speed])
            #     elif am == 1:
            #         self.send([-self.speed, self.speed, -self.speed, self.speed])
            #     else:
            #         assert am == 2
            #         self.send([self.speed, -self.speed, self.speed, -self.speed])
            r.sleep()

    def turn_off_motors(self):
        """
        Turns off all motors.
        """
        self.LOGGER.info("Stopping motors...")
        self.turn_motors([0, 0, 0, 0])

    def turn_motors(self, values):
	"""
	ACTUALLY turns the motors.
	"""

	if self.motors == values or len(values) is not 4:
	    return

	self.LOGGER.info("Motors turning: " + str(values))

	while self.motors[0] != values[0] and self.motors[1] != values[1] and self.motors[2] != values[2] and self.motors[3] != values[3]:
	    for idx, motor_speed in enumerate(values):
                direction = Adafruit_MotorHAT.FORWARD if self.motors[idx] > 0 else Adafruit_MotorHAT.BACKWARD if self.motors[idx] < 0 else Adafruit_MotorHAT.BRAKE
                self.controller.getMotor(idx + 1).run(direction)
                self.motors[idx] += (1 if self.motors[idx] < values[idx] else -1 if self.motors[idx] > values[idx] else 0)
                self.controller.getMotor(idx + 1).setSpeed(int(abs(self.motors[idx])))
            time.sleep(0.001)

    def on_motor_callback(self, value_str_obj):
        """
        Performs the actual motor operations.
        """

	values = list(map(lambda x: float(x), value_str_obj.data.replace("\\", "").replace(" ", "").split(",")))
	self.turn_motors(values)
        self.motors = values

class Wheel(Enum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3

Interface()
