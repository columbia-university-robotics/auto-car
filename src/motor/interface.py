#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import sys
import atexit
import rospy
import numpy as np
import time

from enum import Enum
from os.path import join, dirname

sys.path.append(join(dirname(__file__), '../..'))
from src.util.logger import Logger

from std_msgs.msg import Int16MultiArray
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
        self.publisher = rospy.Publisher('/motor/pub', Int16MultiArray, queue_size=1, latch=True)
        # Initialize motor subscribers
        rospy.init_node('interface')
        rospy.Subscriber("/motor", Int16MultiArray, self.on_motor_callback)
        # Turn off motors when the script exits.
        atexit.register(self.turn_off_motors)

        # motor tracking
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.LOGGER.info(str(self.motors))
            r.sleep()

    def turn_off_motors(self):
        """
        Turns off all motors.
        """
        self.LOGGER.info("Stopping motors...")
        self.send([0, 0, 0, 0])

    def send(self, values):
        """
        Performs motor movement through the publisher.
        """
        self.LOGGER.info("Publish [%s]" % values)
        msg = Int16MultiArray()
        msg.data = values
        self.publisher.publish(msg)

    def on_motor_callback(self, values):
        """
        Performs the actual motor operations.
        """
        assert len(values) is 4
        self.motors = values

        for idx, motor_speed in enumerate(values):
            if motor_speed > 0:
                direction = Adafruit_MotorHAT.FORWARD
            elif motor_speed < 0:
                direction = Adafruit_MotorHAT.BACKWARD
            else:
                direction = Adafruit_MotorHAT.BRAKE
            mh.getMotor.setSpeed(abs(motor_speed * 255))
            mh.getMotor(idx + 1).run(direction)


class Wheel(Enum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3


if __name__ is '__main__':
    Interface()
