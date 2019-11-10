#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import atexit
import rospy
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray

from src.util.logger import Logger


class Interface:

    """
    Abstract:
    Simple script to baseline motor execution.
    """

    def __init__(self, speed = 1):
        self.LOGGER = Logger(self)
        self.motors = [0, 0, 0, 0]
        self.speed = speed
        self.publisher = rospy.Publisher('/bot/hat_cmd',
                                  Int16MultiArray, queue_size=1,
                                  latch=True)
        # Initialize sonar subscribers
        rospy.init_node('sonar_subscriber')
        for i in range(3):
            rospy.Subscriber("/bot/sonar_%d" % i, Range, self.callback, i)
        # Turn off motors when the script exits.
        atexit.register(self.turn_off_motors)

        # Sonar tracking
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.LOGGER.info(str(self.motors))
            if self.motors[0] is self.motors[1] and self.motors[1] is self.motors[2]:
                self.send([0, 0, 0, 0])
            else:
                am = np.argmax(self.motors)
                self.LOGGER.info("AM " + am)
                if am == 0:
                    self.send([self.speed, self.speed, self.speed, self.speed])
                elif am == 1:
                    self.send([-self.speed, self.speed, -self.speed, self.speed])
                else:
                    assert am == 2
                    self.send([self.speed, -self.speed, self.speed, -self.speed])
            r.sleep()

    def turn_off_motors(self):
        """
        Turns off all motors.
        """
        self.LOGGER.info("Stopping motors...")
        self.send([0, 0, 0, 0])

    def callback(self, msg, idx):
        self.motors[idx] = msg.range

    def send(self, values):
        values = [ -v for v in values ]
        rospy.loginfo("publish [%s]" % values)
        msg = Int16MultiArray()
        msg.data = values
        self.publisher.publish(msg)

