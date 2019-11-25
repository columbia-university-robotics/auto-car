#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import os
import sys
import rospy
from std_msgs.msg import String

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.util.logger import Logger


class Control:

    """
    Abstract:
    Publishes to the ROS motor interface subscriber to move the car in preset directions.
    """

    def __init__(self, base_speed=126):
        # Base speed of the car between -255 and 255
        self.base_speed = base_speed
        # Array of magnitudes of the speed of each motor between -1 and 1. Makes changing speeds easy.
        self.speed_mags = [0, 0, 0, 0]
        # Publisher to the ROS motor interface
        self.pub = rospy.Publisher('chatter', String, queue_size=3, latch=False)
        self.LOGGER = Logger(self)

    def __move(self, mags):
        """
        Moves motors in the specified directions between speed magnitudes of -1 and 1.
        :param mags: Array of 4 motor speed magnitudes corresponding to motor 1, motor 2, motor 3, and motor 4.
        """
        if len(mags) is 4 and all(isinstance(m, int) and abs(m) <= 1 for m in mags):
            self.speed_mags = mags
            values = self.base_speed * mags
            self.pub(",".join([str(v) for v in values]))
        else:
            self.LOGGER("Could not move motors due to invalid motor magnitudes: " + str(mags))

    """
    Simple Movement
    """

    def set_speed(self, speed):
        """
        Sets the base speed of the car.
        :param speed: Speed of the car between -255 and 255.
        """
        if isinstance(speed, int) and abs(speed) <= 255:
            self.base_speed = speed
            self.__move(self.speed_mags)
        else:
            self.LOGGER("Could not change base speed due to invalid speed: " + str(speed))

    def stop(self):
        """
        Stops all movement.
        """
        self.LOGGER("Stopping")
        self.__move([0, 0, 0, 0])

    def go(self):
        """
        Continue straight.
        """
        self.LOGGER("Moving forward")
        # TODO

    def back(self):
        """
        Move in the reverse direction.
        """
        self.LOGGER("Moving backward")
        # TODO

    """
    Crab Movement
    """

    def crab_l(self):
        """
        More laterally/horizontally in the left direction.
        """
        self.LOGGER("Moving laterally left")
        # TODO

    def crab_r(self):
        """
        More laterally/horizontally in the right direction.
        """
        self.LOGGER("Moving laterally right")
        # TODO

    """
    Simple Turning Movement
    """

    def turn_l(self):
        """
        Performs a 90deg rotation left.
        """
        self.LOGGER("Turning left")
        # TODO

    def turn_r(self):
        """
        Performs a 90deg rotation right.
        """
        self.LOGGER("Turning right")
        # TODO

    """
    Rotation Movement
    """

    def rot_l(self):
        """
        Performs a 180deg rotation left.
        """
        self.LOGGER("Rotating left")
        # TODO

    def rot_r(self):
        """
        Performs a 180deg rotation right.
        """
        self.LOGGER("Rotating right")
        # TODO

    """
    Spin Movement
    """

    def spin_l(self):
        """
        Spins 360deg left.
        """
        self.LOGGER("Spinning left")
        # TODO

    def spin_r(self):
        """
        Performs 360deg right.
        """
        self.LOGGER("Spinning right")
        # TODO

