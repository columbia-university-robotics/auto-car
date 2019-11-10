#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

from src.util.logger import Logger
from src.motor.interface import Interface


class Control:

    """
    Abstract:
    Uses the interface class to implement turning functions.
    """

    def __init__(self):
        self.LOGGER = Logger(self)

    """
    Simple Movement
    """

    def stop(self):
        """
        Stops all movement.
        """
        self.LOGGER("Stopping")
        # TODO

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
    Lateral Movement
    """

    def lat_l(self):
        """
        More laterally/horizontally in the left direction.
        """
        self.LOGGER("Moving laterally left")
        # TODO

    def lat_r(self):
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

