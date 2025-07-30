#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达数据处理器
- 处理原始激光雷达数据
- 进行聚类分析
- 检测锥筒候选点
"""
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from fsd_common_msgs.msg import ConeDetections, Cone
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Float32

class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor', anonymous=True)
        
        # 参数
        self.max_range = rospy.get_param('~max_range', 25.0)
        self.min_range = rospy.get_param('~min_range', 0.2)
        self.cone_detection_distance = rospy.get_param('~cone_detection_distance', 15.0)
        self.cluster_tolerance = rospy.get_param('~cluster_tolerance', 0.4)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 3)
        self.max_cluster_size = rospy.get_param('~max_cluster_size', 50)
        
        # 订阅器
        self.scan_sub = rospy.Subscriber('/perception/lidar/scan', LaserScan, self.scan_callback, queue_size=10)
        
        # 发布器
        self.clusters_pub = rospy.Publisher('/perception/lidar/clusters', ConeDetections, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/perception/lidar/pointcloud', PointCloud2, queue_size=10)
        
        rospy.loginfo("激光雷达处理器启动完成")

    def scan_callback(self, scan_msg):
        """激光雷达数据回调"""
        try:
            # 转换为点云
            points = self.scan_to_points(scan_msg)
            
            # 发布点云（用于可视化）
            self.publish_pointcloud(points, scan_msg.header)
            
            # 进行聚类
            clusters = self.cluster_points(points)
            
            # 发布聚类结果
            self.publish_clusters(clusters, scan_msg.header)
            
        except Exception as e:
            rospy.logwarn(f"激光雷达处理出错: {e}")

    def scan_to_points(self, scan_msg):
        """将激光雷达扫描转换为2D点"""
        points = []
        
        for i, range_val in enumerate(scan_msg.ranges):
            # 过滤无效点
            if (self.min_range <= range_val <= self.max_range and 
                range_val <= self.cone_detection_distance):
                
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append([x, y])
        
        return np.array(points)

    def cluster_points(self, points):
        """对点进行聚类分析"""
        if len(points) == 0:
            return []
        
        clusters = []
        visited = [False] * len(points)
        
        for i in range(len(points)):
            if visited[i]:
                continue
            
            # 开始新的聚类
            cluster = []
            stack = [i]
            
            while stack:
                current_idx = stack.pop()
                if visited[current_idx]:
                    continue
                
                visited[current_idx] = True
                cluster.append(points[current_idx])
                
                # 查找邻近点
                for j in range(len(points)):
                    if not visited[j]:
                        distance = np.linalg.norm(points[current_idx] - points[j])
                        if distance <= self.cluster_tolerance:
                            stack.append(j)
            
            # 检查聚类大小
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                clusters.append(np.array(cluster))
        
        return clusters

    def publish_pointcloud(self, points, header):
        """发布点云数据"""
        if len(points) == 0:
            return
        
        # 创建点云消息
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        # 添加z=0的坐标
        points_3d = []
        for point in points:
            points_3d.append([point[0], point[1], 0.0])
        
        pc_msg = pc2.create_cloud(header, fields, points_3d)
        self.pointcloud_pub.publish(pc_msg)

    def publish_clusters(self, clusters, header):
        """发布聚类结果"""
        detection_msg = ConeDetections()
        detection_msg.header = header
        
        for cluster in clusters:
            # 计算聚类中心
            center = np.mean(cluster, axis=0)
            
            # 创建锥筒检测消息
            cone = Cone()
            cone.position.x = center[0]
            cone.position.y = center[1]
            cone.position.z = 0.0
            
            # 修改这里：使用字符串而不是常量
            from std_msgs.msg import String, Float32
            cone.color = String()
            cone.color.data = 'unknown'  # 使用字符串
            
            cone.poseConfidence = Float32()
            cone.poseConfidence.data = 0.5
            
            cone.colorConfidence = Float32()
            cone.colorConfidence.data = 0.0  # 未知颜色，置信度为0
            
            # 修改这里：使用正确的字段名
            detection_msg.cone_detections.append(cone)
        
        # 如果有cone_count字段的话
        if hasattr(detection_msg, 'cone_count'):
            detection_msg.cone_count = len(detection_msg.cone_detections)
        
        self.clusters_pub.publish(detection_msg)

if __name__ == '__main__':
    try:
        processor = LidarProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("激光雷达处理器正常退出")