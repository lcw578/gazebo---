#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: src/fsac_autonomous/scripts/control/cmd_vel_converter.py
"""
Ackermann指令到差分驱动指令的转换器
"""
import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

class CmdVelConverter:
    def __init__(self):
        rospy.init_node('cmd_vel_converter', anonymous=True)
        
        # 车辆参数
        self.wheelbase = 0.3302
        self.wheel_separation = 0.2
        
        # 订阅Ackermann指令
        self.ackermann_sub = rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, 
                                            self.ackermann_callback, queue_size=10)
        
        # 发布差分驱动指令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Ackermann到差分驱动转换器启动")

    def ackermann_callback(self, msg):
        """转换Ackermann指令为cmd_vel"""
        twist = Twist()
        
        # 修正速度方向 - 取反
        twist.linear.x = -msg.drive.speed  # 添加负号
        twist.angular.z = msg.drive.steering_angle * 0.5
        
        self.cmd_vel_pub.publish(twist)
        
        # 添加调试日志
        rospy.loginfo_throttle(2.0, 
            f"转换: Ackermann速度={msg.drive.speed:.2f} -> cmd_vel={twist.linear.x:.2f}")

if __name__ == '__main__':
    try:
        converter = CmdVelConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("转换器退出")