#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import os
import sys
import atexit
import rospy
import time
from std_msgs.msg import String

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.util.logger import Logger
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor


# Rate at which ROS should track newly published commands
ROS_TRACKING_RATE_S = 0.5


class Interface:

    """
    Abstract:
    Simple script that subscribes to motor commands and processes them.
    """

    # TODO: Update these values for the motor controller's specs
    # Maps motor values from the array to their actual numerical values
    MAPPING = {
        # Motor 1
        0: 1,
        # Motor 2
        1: 2,
        # Motor 3
        2: 3,
        # Motor 4
        3: 4
    }

    def __init__(self):
        """
        Initializes the motor interface.
        """
        self.LOGGER = Logger(self)
        self.controller = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
        self.motors = [0, 0, 0, 0]
        # Initialize motor subscribers
        rospy.init_node('interface')
        rospy.Subscriber("/motor", String, self.on_motor_callback, queue_size=1)
        # Turn off motors when the script exits.
        atexit.register(self.turn_off_motors)
        # Motor tracking
        r = rospy.Rate(ROS_TRACKING_RATE_S)
        while not rospy.is_shutdown():
            self.LOGGER.info(str(self.motors))
            r.sleep()

    def turn_off_motors(self):
        """
        Turns off all motors.
        """
        self.LOGGER.info("Stopping motors...")
        self.turn_motors([0, 0, 0, 0], 1000000)

    def turn_motors(self, values, accel=10000):
        """
        Turns motors to the given values with the provided acceleration.
        :param values Array of 4 motor values between -255 and 255 each, indicating the
                        speed of each motor ([ MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4 ]).
        :param accel The feaux "acceleration" of the motors, in Hz.
        """
        if self.motors == values or len(values) is not 4:
            # Return if not all 4 motor values are provided
            return

        self.LOGGER.info("Motors turning: " + str(values))

        # Slowly increment the motors with the provided acceleration.
        while self.motors[0] != values[0] and self.motors[1] != values[1] and self.motors[2] != values[2] and self.motors[3] != values[3]:
            for idx, motor_speed in enumerate(values):
                self.motors[idx] += (1 if self.motors[idx] < values[idx] else -1 if self.motors[idx] > values[idx] else 0)
                direction = Adafruit_MotorHAT.FORWARD if self.motors[idx] > 0 else Adafruit_MotorHAT.BACKWARD if self.motors[idx] < 0 else Adafruit_MotorHAT.BRAKE
                self.controller.getMotor(self.MAPPING[idx]).run(direction)
                self.controller.getMotor(self.MAPPING[idx]).setSpeed(int(abs(self.motors[idx])))
                time.sleep(1/accel)

    def on_motor_callback(self, value_str_obj):
        """
        Takes in a ROS String and converts it to a list of motor values.
        Callback for ROS /motor subscription.
        """
        values = list(map(lambda x: float(x), value_str_obj.data.replace("\\", "").replace(" ", "").split(",")))
        self.turn_motors(values)
        self.motors = values


Interface()
