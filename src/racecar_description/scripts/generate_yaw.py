#!/usr/bin/env python
import rospy
import random
import subprocess

def generate_yaw_and_spawn():
    # Generate a random yaw angle between -3 and 3 degrees
    yaw = random.uniform(-3, 3)
    rospy.set_param('/yaw', yaw)

    # Convert yaw to radians
    yaw_rad = yaw * 3.14159 / 180.0

    # Spawn the racecar model in Gazebo
    subprocess.call([
        "rosrun", "gazebo_ros", "spawn_model",
        "-param", "robot_description",
        "-urdf", "-model", "racecar",
        "-x", "-0.3", "-y", "0", "-z", "0.1",
        "-Y", str(yaw_rad)
    ])

if __name__ == '__main__':
    rospy.init_node('generate_yaw_and_spawn')
    generate_yaw_and_spawn()
