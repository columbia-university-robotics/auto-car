#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

from enum import Enum
from time import gmtime, strftime

from termcolor import cprint

class Logger:
    """
    Abstract:
    A class to enable class-based logging, like in Java (see Logger4J).
    """

    def __init__(self, cls=None):
        self.cls = cls

    def info(self, *msgs):
        """
        Prints the provided messages, with timestamps, for debugging purposes.
        :param msgs: List of messages.
        """
        for msg in msgs:
            print(self.__prefix() + msg)

    def warn(self, *msgs):
        """
        Prints the provided messages, with timestamps, as warnings.
        :param msgs: List of messages.
        """
        for msg in msgs:
            cprint(self.__prefix() + msg, TerminalColor.WARNING.value)

    def error(self, *msgs):
        """
        Prints the provided messages, with timestamps, as errors.
        :param msgs: List of messages.
        """
        for msg in msgs:
            cprint(self.__prefix() + msg, TerminalColor.FAIL.value)

    def __prefix(self):
        """
        :return: A combintation of class name and timestamp, for convenience.
        """
        return self.__classname() + " [" + self.__timestamp() + "]: "

    def __classname(self):
        """
        :return: Logger's class name as a string.
        """
        return self.cls.__class__.__name__ if self.cls is not None else "No-Class"

    @staticmethod
    def __timestamp():
        """
        :return: Current UTC timestamp as a string.
        """
        return str(strftime("%H:%M:%S", gmtime()))


class TerminalColor(Enum):
    BLUE = 'blue'
    GREEN = 'green'
    WARNING = 'yellow'
    FAIL = 'red'
