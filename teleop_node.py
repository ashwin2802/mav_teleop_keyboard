#!/usr/bin/env python

from __future__ import print_function
import tty
import termios
import select
import sys
import math
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import rospy

import threading
import roslib
roslib.load_manifest('mav_teleop_keyboard')


init_msg = """
Reading from the keyboard...
-----------------------------
Move forward:           w
Move backward:          s

Move left:          a
Move right:         d

Increase Thrust:        q
Decrease Thrust:        e

Increase speed by 10%:          k
Decrease speed by 10%:          l

Anything else:          Stop

Ctrl-C to quit
"""

keyBindings = {
    'a': (1, 0, 0),
    'd': (-1, 0, 0),
    'w': (0, 1, 0),
    's': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}

speedBindings = {
    'k': 1.1,
    'l': 0.9
}


class PublisherThread(threading.Thread):
    def __init__(self, rate):
        super(PublisherThread, self).__init__()
        self.publisher = rospy.Publisher(
            "command/pose", PoseStamped, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            "ground_truth/odometry", Odometry, self.poseCallback)
        self.condition = threading.Condition()
        self.done = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.speed = 1
        self.curr_pose = Pose()

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def poseCallback(self, data):
        self.curr_pose = data.pose.pose

    def wait_for_subscribers(self):
        count = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if (count == 4):
                print("Waiting for subscriber to connect to {}".format(
                    self.publisher.name))
            rospy.sleep(0.5)
            count += 1
            count = count % 5
        if rospy.is_shutdown():
            raise Exception("Shutdown before receiving a connection")

    def update(self, x, y, z, speed):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.speed = speed
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 1)
        self.join()

    def run(self):
        pose_cmd = PoseStamped()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            pose_cmd.pose.position.x = self.curr_pose.position.x + self.x * self.speed
            pose_cmd.pose.position.y = self.curr_pose.position.y + self.y * self.speed
            pose_cmd.pose.position.z = self.curr_pose.position.z + self.z * self.speed

            self.condition.release()
            self.publisher.publish(pose_cmd)

        pose_cmd.pose.position = self.curr_pose.position
        self.publisher.publish(pose_cmd)


def print_speed(speed):
    print("Currently speed is: {}".format(speed))


def getKeystroke(timeout):
    tty.setraw(sys.stdin.fileno())
    rready, _, _ = select.select([sys.stdin], [], [], timeout)
    if rready:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # print("Got key:{}".format(key))
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('mav_teleop_keyboard')

    publish_thread = PublisherThread(0.0)

    x = 0
    y = 0
    z = 0
    speed = 0.5

    try:
        publish_thread.wait_for_subscribers()
        publish_thread.update(x, y, z, speed)

        print(init_msg)
        print_speed(speed)

        while (1):
            key = getKeystroke(1.0)
            if key in keyBindings.keys():
                x, y, z = keyBindings[key]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]
                print_speed(speed)
            else:
                if key == '' and x == 0 and y == 0 and z == 0:
                    continue
                x = 0
                y = 0
                z = 0
                if (key == '\x03'):
                    break

            publish_thread.update(x, y, z, speed)

    except Exception as e:
        print(e)

    finally:
        publish_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
