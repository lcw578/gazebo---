#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: e:\gazebo大作业\gazebo---\catkin_ws\src\fsac_autonomous\scripts\perception\complete_cone_system.py
"""
完整的锥筒检测和可视化系统
解决话题映射和数据流问题
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from fsd_common_msgs.msg import ConeDetections, Cone
from std_msgs.msg import String, Float32, Header

class CompleteConeSystem:
    def __init__(self):
        rospy.init_node('complete_cone_system', anonymous=True)
        
        # 订阅激光雷达（尝试多个可能的话题）
        self.scan_sub1 = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        self.scan_sub2 = rospy.Subscriber('/perception/lidar/scan', LaserScan, self.scan_callback, queue_size=10)
        
        # 发布锥筒检测
        self.cone_detections_pub = rospy.Publisher('/perception/lidar/cone_detections', 
                                                  ConeDetections, queue_size=10)
        
        # 发布可视化
        self.red_cones_pub = rospy.Publisher('/mapping/red_cones', MarkerArray, queue_size=100)
        self.blue_cones_pub = rospy.Publisher('/mapping/blue_cones', MarkerArray, queue_size=100)
        self.yellow_cones_pub = rospy.Publisher('/mapping/yellow_cones', MarkerArray, queue_size=100)
        
        # 检测参数
        self.min_range = 1.0
        self.max_range = 15.0
        self.cluster_tolerance = 0.5
        self.min_cluster_size = 2
        
        rospy.loginfo("🎯 完整锥筒系统启动")

    def scan_callback(self, scan_msg):
        """激光雷达回调"""
        try:
            # 检测锥筒
            cone_positions = self.detect_cones_from_scan(scan_msg)
            
            if cone_positions:
                # 发布锥筒检测消息
                cone_detections = self.create_cone_detections_msg(cone_positions, scan_msg.header)
                self.cone_detections_pub.publish(cone_detections)
                
                # 发布可视化
                self.publish_visualization(cone_positions)
                
                rospy.loginfo_throttle(3.0, f"检测到 {len(cone_positions)} 个锥筒")
            
        except Exception as e:
            rospy.logwarn(f"锥筒检测失败: {e}")

    def detect_cones_from_scan(self, scan_msg):
        """从激光雷达数据检测锥筒"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # 过滤有效数据
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range) & np.isfinite(ranges)
        
        if not np.any(valid_mask):
            return []
        
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # 转换为笛卡尔坐标
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # 简单聚类检测
        cones = []
        used = np.zeros(len(x), dtype=bool)
        
        for i in range(len(x)):
            if used[i]:
                continue
            
            # 寻找附近的点
            cluster_x = [x[i]]
            cluster_y = [y[i]]
            used[i] = True
            
            # 聚类
            for j in range(i+1, len(x)):
                if used[j]:
                    continue
                
                dist = math.sqrt((x[i] - x[j])**2 + (y[i] - y[j])**2)
                if dist < self.cluster_tolerance:
                    cluster_x.append(x[j])
                    cluster_y.append(y[j])
                    used[j] = True
            
            # 如果簇足够大，认为是锥筒
            if len(cluster_x) >= self.min_cluster_size:
                center_x = np.mean(cluster_x)
                center_y = np.mean(cluster_y)
                
                # 根据位置确定颜色
                if center_y > 0.5:
                    color = 'red'    # 左侧红色
                elif center_y < -0.5:
                    color = 'blue'   # 右侧蓝色
                else:
                    color = 'yellow' # 中间黄色
                
                cones.append({
                    'x': center_x,
                    'y': center_y,
                    'color': color
                })
        
        return cones

    def create_cone_detections_msg(self, cone_positions, header):
        """创建锥筒检测消息"""
        detection_msg = ConeDetections()
        detection_msg.header = header
        
        for cone_pos in cone_positions:
            cone = Cone()
            cone.position.x = cone_pos['x']
            cone.position.y = cone_pos['y']
            cone.position.z = 0.0
            
            # 设置颜色
            cone.color = String()
            cone.color.data = cone_pos['color']
            
            # 设置置信度
            cone.poseConfidence = Float32()
            cone.poseConfidence.data = 0.8
            
            cone.colorConfidence = Float32()
            cone.colorConfidence.data = 0.7
            
            detection_msg.cone_detections.append(cone)
        
        return detection_msg

    def publish_visualization(self, cone_positions):
        """发布可视化"""
        # 分类锥筒
        red_cones = [c for c in cone_positions if c['color'] == 'red']
        blue_cones = [c for c in cone_positions if c['color'] == 'blue']
        yellow_cones = [c for c in cone_positions if c['color'] == 'yellow']
        
        # 发布各颜色锥筒
        self.publish_cone_markers(red_cones, 'red', self.red_cones_pub)
        self.publish_cone_markers(blue_cones, 'blue', self.blue_cones_pub)
        self.publish_cone_markers(yellow_cones, 'yellow', self.yellow_cones_pub)

    def publish_cone_markers(self, cones, color, publisher):
        """发布锥筒标记"""
        marker_array = MarkerArray()
        
        # 清空之前的标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 颜色设置
        colors = {
            'red': (1.0, 0.0, 0.0),
            'blue': (0.0, 0.0, 1.0),
            'yellow': (1.0, 1.0, 0.0)
        }
        
        if not cones:
            publisher.publish(marker_array)
            return
        
        r, g, b = colors.get(color, (0.5, 0.5, 0.5))
        
        for i, cone in enumerate(cones):
            marker = Marker()
            marker.header.frame_id = "base_link"  # 使用激光雷达坐标系
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"{color}_cones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 位置
            marker.pose.position.x = cone['x']
            marker.pose.position.y = cone['y']
            marker.pose.position.z = 0.5
            
            # 方向
            marker.pose.orientation.w = 1.0
            
            # 大小
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            
            # 颜色
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.8
            
            # 生存时间
            marker.lifetime = rospy.Duration(1.0)
            
            marker_array.markers.append(marker)
        
        publisher.publish(marker_array)

if __name__ == '__main__':
    try:
        system = CompleteConeSystem()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass