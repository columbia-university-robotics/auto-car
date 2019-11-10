#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

from enum import Enum
from time import gmtime, strftime


class Logger:
    """
    Abstract:
    A class to enable class-based logging, like in Java (see Logger4J).
    """

    def __init__(self, cls):
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
            print(TerminalColor.WARNING + self.__prefix() + msg + TerminalColor.END)

    def error(self, *msgs):
        """
        Prints the provided messages, with timestamps, as errors.
        :param msgs: List of messages.
        """
        for msg in msgs:
            print(TerminalColor.FAIL + self.__prefix() + msg + TerminalColor.END)

    def __prefix(self):
        """
        :return: A combintation of class name and timestamp, for convenience.
        """
        return self.__classname() + " [" + self.__timestamp() + "]: "

    def __classname(self):
        """
        :return: Logger's class name as a string.
        """
        return self.cls.__class__.__name__

    @staticmethod
    def __timestamp():
        """
        :return: Current UTC timestamp as a string.
        """
        return str(strftime("%H:%M:%S", gmtime()))


class TerminalColor(Enum):
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'