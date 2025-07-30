#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
锥筒检测与分类器
- 接收聚类数据
- 分类锥筒（蓝色/黄色）
- 过滤噪声
"""
import rospy
import math
import numpy as np
from fsd_common_msgs.msg import ConeDetections, Cone
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32

class ConeDetector:
    def __init__(self):
        rospy.init_node('cone_detector', anonymous=True)
        
        # 参数
        self.detection_range = rospy.get_param('~detection_range', 20.0)
        self.cone_height_threshold = rospy.get_param('~cone_height_threshold', 0.3)
        self.classification_enabled = rospy.get_param('~classification_enabled', True)
        self.min_points_per_cone = rospy.get_param('~min_points_per_cone', 3)
        self.max_cone_width = rospy.get_param('~max_cone_width', 0.5)
        
        # 订阅器
        self.clusters_sub = rospy.Subscriber('/perception/lidar/clusters', ConeDetections, 
                                           self.clusters_callback, queue_size=10)
        
        # 发布器 - 修改为dynamic_mapper期望的话题
        self.cones_pub = rospy.Publisher('/perception/lidar/cone_detections', ConeDetections, queue_size=10)
        
        rospy.loginfo("锥筒检测器启动完成")

    def clusters_callback(self, clusters_msg):
        """聚类数据回调"""
        try:
            # 修改这里：使用正确的字段名 cone_detections
            cone_positions = [cone.position for cone in clusters_msg.cone_detections]
            
            if len(cone_positions) == 0:
                return
            
            # 检测锥筒颜色
            detections = self.detect_cone_colors(cone_positions)
            
            # 发布结果
            self.cones_pub.publish(detections)
            
        except Exception as e:
            rospy.logwarn(f"锥筒检测出错: {e}")

    def filter_and_classify_cones(self, raw_cones):
        """过滤和分类锥筒"""
        filtered_cones = []
        
        for cone in raw_cones:
            # 距离过滤
            distance = math.sqrt(cone.position.x**2 + cone.position.y**2)
            if distance > self.detection_range:
                continue
            
            # 基本有效性检查
            if not self.is_valid_cone(cone):
                continue
            
            # 分类锥筒颜色
            if self.classification_enabled:
                cone.color = self.classify_cone_color(cone)
            
            filtered_cones.append(cone)
        
        return filtered_cones

    def is_valid_cone(self, cone):
        """检查锥筒是否有效"""
        # 检查位置是否合理
        if abs(cone.position.x) < 0.5:  # 太近的点可能是噪声
            return False
        
        # 可以添加更多有效性检查
        return True

    def classify_cone_color(self, cone):
        """分类锥筒颜色（简化版本）"""
        # 这里使用简单的位置规则进行分类
        # 实际应用中可能需要更复杂的算法
        
        if cone.position.y > 0:
            return Cone.BLUE   # 左侧蓝色
        else:
            return Cone.YELLOW # 右侧黄色

    def publish_cones(self, cones, header):
        """发布锥筒检测结果"""
        detection_msg = ConeDetections()
        detection_msg.header = header
        detection_msg.cones = cones
        detection_msg.cone_count = len(cones)
        
        self.cones_pub.publish(detection_msg)
        
        # 记录检测到的锥筒数量
        blue_count = sum(1 for cone in cones if cone.color == Cone.BLUE)
        yellow_count = sum(1 for cone in cones if cone.color == Cone.YELLOW)
        
        rospy.loginfo_throttle(2.0, f"检测到锥筒: 蓝色{blue_count}个, 黄色{yellow_count}个")

    def detect_cone_colors(self, cone_positions):
        """检测锥筒颜色"""
        detections = ConeDetections()
        detections.header.stamp = rospy.Time.now()
        detections.header.frame_id = "base_link"
        
        for pos in cone_positions:
            cone = Cone()
            cone.position = pos
            
            # 修改颜色检测逻辑
            from std_msgs.msg import String, Float32
            cone.color = String()
            cone.poseConfidence = Float32()
            cone.colorConfidence = Float32()
            
            # 简单的距离分类逻辑
            if pos.y > 0:  # 左侧
                cone.color.data = 'r'  # 红色
                cone.colorConfidence.data = 0.8
            elif pos.y < 0:  # 右侧
                cone.color.data = 'b'  # 蓝色
                cone.colorConfidence.data = 0.8
            else:
                cone.color.data = 'unknown'
                cone.colorConfidence.data = 0.0
                
            cone.poseConfidence.data = 0.9
            # 修改这里：使用正确的字段名
            detections.cone_detections.append(cone)
        
        if hasattr(detections, 'cone_count'):
            detections.cone_count = len(detections.cone_detections)
        
        return detections

if __name__ == '__main__':
    try:
        detector = ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("锥筒检测器正常退出")