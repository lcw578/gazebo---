#!/usr/bin/env python
import rospy
import random

def generate_yaw():
    # Generate a random yaw angle between -3 and 3 degrees
    yaw = random.uniform(-3, 3)
    rospy.set_param('/yaw', yaw)

if __name__ == '__main__':
    rospy.init_node('generate_yaw')
    generate_yaw()
    