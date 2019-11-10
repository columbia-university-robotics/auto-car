#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import sys
import slack
import socket
from os import environ
from os.path import join, dirname
from dotenv import load_dotenv

sys.path.append(join(dirname(__file__), '../..'))
from src.util.logger import Logger


# Load Slack API Token
load_dotenv(join(join(dirname(__file__), '../..'), '.env'))
TOKEN = environ.get("SLACK_API_TOKEN")

# Create Slack client
client = slack.WebClient(token=TOKEN)

LOGGER = Logger()


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]

def notify_ip():
    """
    Notifies the Slack channel of the car's IP address.
    """
    ip = get_ip_address()
    LOGGER.info("IP: " + ip)
    LOGGER.info("Token: " + TOKEN)
    response = client.chat_postMessage(channel='#autonomy-bot', text="Hi! SSH into me with `ssh pi@" + ip + "`.")
    assert response["ok"]
    LOGGER.info("Notified")


if __name__ == "__main__":
    notify_ip()

