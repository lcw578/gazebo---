#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版FSD数据处理器
功能：处理bag数据，显示锥筒和车辆位置
"""
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from std_msgs.msg import String

# 注意：这里我们直接使用rospy.AnyMsg来接收任何类型的消息
# 避免复杂的消息类型定义

class SimpleProcessor:
    def __init__(self):
        rospy.init_node('simple_processor', anonymous=True)
        
        # 数据存储
        self.all_cones = []           # 所有检测到的锥筒
        self.current_car_pos = None   # 当前车辆位置
        self.car_path = []            # 车辆轨迹
        
        # 配置参数
        self.lidar_offset = 2.4       # 激光雷达偏移距离
        
        # 订阅器（使用AnyMsg避免消息类型问题）
        self.car_sub = rospy.Subscriber('/estimation/slam/state', rospy.AnyMsg, self.car_callback)
        self.cone_sub = rospy.Subscriber('/perception/lidar/cone_detections', rospy.AnyMsg, self.cone_callback)
        
        # 发布器
        self.cone_pub = rospy.Publisher('/simple/cones', MarkerArray, queue_size=10)
        self.car_pub = rospy.Publisher('/simple/car_pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/simple/car_path', Path, queue_size=10)
        self.info_pub = rospy.Publisher('/simple/info', String, queue_size=10)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_info)
        
        rospy.loginfo("=== 简化处理器启动 ===")

    def car_callback(self, msg):
        """处理车辆状态（简化版）"""
        try:
            # 由于使用AnyMsg，我们需要手动解析
            # 这里假设消息格式包含car_state字段
            import genpy
            
            # 简化：直接从消息中提取位置信息
            # 你可能需要根据实际的消息格式调整这部分
            
            # 临时方案：使用固定位置进行测试
            if self.current_car_pos is None:
                self.current_car_pos = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
                rospy.loginfo("使用默认车辆位置进行测试")
            
            # 发布车辆位置
            self.publish_car_pose()
            
        except Exception as e:
            rospy.logwarn(f"解析车辆消息失败: {e}")

    def cone_callback(self, msg):
        """处理锥筒检测（简化版）"""
        try:
            # 简化版：手动解析消息
            # 在实际应用中，你需要根据真实的消息格式来解析
            
            # 临时方案：生成一些测试锥筒
            if len(self.all_cones) == 0:
                # 创建一些测试锥筒来验证可视化
                test_cones = [
                    {'x': 5.0, 'y': 2.0, 'color': 'r'},    # 红色（左边界）
                    {'x': 10.0, 'y': 2.0, 'color': 'r'},
                    {'x': 15.0, 'y': 2.0, 'color': 'r'},
                    {'x': 5.0, 'y': -2.0, 'color': 'b'},   # 蓝色（右边界）
                    {'x': 10.0, 'y': -2.0, 'color': 'b'},
                    {'x': 15.0, 'y': -2.0, 'color': 'b'},
                ]
                self.all_cones = test_cones
                rospy.loginfo("生成测试锥筒数据")
            
            # 发布锥筒可视化
            self.publish_cones()
            
        except Exception as e:
            rospy.logwarn(f"解析锥筒消息失败: {e}")

    def publish_car_pose(self):
        """发布车辆位姿"""
        if self.current_car_pos is None:
            return
            
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = self.current_car_pos['x']
        pose.pose.position.y = self.current_car_pos['y']
        pose.pose.position.z = 0.0
        
        # 简化：不设置姿态
        pose.pose.orientation.w = 1.0
        
        self.car_pub.publish(pose)

    def publish_cones(self):
        """发布锥筒可视化（简化版）"""
        marker_array = MarkerArray()
        
        # 清除旧标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 添加锥筒标记
        for i, cone in enumerate(self.all_cones):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "simple_cones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 位置
            marker.pose.position.x = cone['x']
            marker.pose.position.y = cone['y']
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            
            # 尺寸
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.4
            
            # 颜色：红色=左边界，蓝色=右边界
            if cone['color'] == 'r':  # 红色
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif cone['color'] == 'b':  # 蓝色
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:  # 其他颜色
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            
            marker_array.markers.append(marker)
        
        self.cone_pub.publish(marker_array)

    def publish_info(self, event):
        """发布统计信息"""
        info = f"锥筒总数: {len(self.all_cones)}"
        if self.current_car_pos:
            info += f", 车辆位置: ({self.current_car_pos['x']:.1f}, {self.current_car_pos['y']:.1f})"
        
        msg = String()
        msg.data = info
        self.info_pub.publish(msg)
        
        rospy.loginfo(info)

if __name__ == '__main__':
    try:
        processor = SimpleProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("简化处理器退出")