#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

from enum import Enum
import atexit
import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray
from adafruit_motorkit import MotorKit

from src.util.logger import Logger


class Interface:

    """
    Abstract:
    Simple script to baseline motor execution.
    """

    def __init__(self, speed = 1):
        self.LOGGER = Logger(self)
        self.kit = MotorKit()
        self.motors = [0, 0, 0, 0]
        self.speed = speed
        self.publisher = rospy.Publisher('/motor/pub', Int16MultiArray, queue_size=1, latch=True)
        # Initialize motor subscribers
        rospy.init_node('interface')
        for i in range(3):
            rospy.Subscriber("/motor", Int16MultiArray, self.on_motor_callback)
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
        kit.motor1.throttle = self.motors[0]
        kit.motor2.throttle = self.motors[1]
        kit.motor3.throttle = self.motors[2]
        kit.motor4.throttle = self.motors[3]


class Wheel(Enum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3
