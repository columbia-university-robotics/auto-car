#!/usr/bin/env python
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Autonomous Car Team.
# Contact: Anthony, anthony.k@columbia.edu

import os
import slack
import socket

"""
The Autonomous Car's Slack API token.
It's not good practice to store tokens directly in code, but hey... :)
"""
TOKEN = "xoxp-778353670119-828326319940-828792273712-15e23853dc3080c05069e6f0e7bdca35"

client = slack.WebClient(token=TOKEN)

def notify_ip():
    """
    Notifies the Slack channel of the car's IP address.
    """
    ip = socket.gethostbyname(socket.gethostname())
    response = client.chat_postMessage(channel='#autonomy', text="Hi! SSH into me with `ssh pi@" + ip + "`.")
    assert response["ok"]


if __name__ == "__main__":
    notify_ip()

