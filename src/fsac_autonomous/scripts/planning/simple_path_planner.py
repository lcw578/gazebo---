#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于预建地图的路径规划器 🗺️➡️🛣️
功能：加载离线地图，为小车提供规划线
"""

import rospy
import pickle
import json
import math
import numpy as np
from scipy import interpolate
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32, String, Bool
from visualization_msgs.msg import Marker, MarkerArray  # 🎨 新增：可视化标记

class MapBasedPlanner:
    def __init__(self):
        rospy.init_node('map_based_planner', anonymous=True)
        
        # 发布器
        self.planned_path_pub = rospy.Publisher('/planning/planned_path', Path, queue_size=10)
        self.target_speed_pub = rospy.Publisher('/planning/target_speed', Float32, queue_size=10)
        self.planner_status_pub = rospy.Publisher('/planning/status', String, queue_size=10)
        self.path_available_pub = rospy.Publisher('/planning/path_available', Bool, queue_size=10)
        
        # 🎨 新增：可视化发布器
        self.planning_markers_pub = rospy.Publisher('/planning/markers', MarkerArray, queue_size=10)
        self.planning_line_pub = rospy.Publisher('/planning/path_line', Marker, queue_size=10)
        self.planning_points_pub = rospy.Publisher('/planning/path_points', MarkerArray, queue_size=10)
        
        # 地图数据
        self.centerline_raw = []
        self.centerline_smooth = []
        self.map_loaded = False
        
        # 规划参数
        self.target_speed = 5.0  # 目标速度
        self.path_resolution = 0.2  # 路径分辨率
        self.smoothing_factor = 0.5  # 平滑因子
        
        # 加载地图
        self.load_map()
        
        if self.map_loaded:
            self.generate_smooth_path()
            
            # 定时发布规划路径
            self.publish_timer = rospy.Timer(rospy.Duration(0.5), self.publish_planned_path)
            # 🎨 定时发布可视化
            self.viz_timer = rospy.Timer(rospy.Duration(1.0), self.publish_visualizations)
            
            rospy.loginfo("🛣️ 地图加载成功，开始提供规划路径")
        else:
            rospy.logfatal("❌ 地图加载失败，无法提供规划路径")

    def load_map(self):
        """加载预建地图"""
        map_files = ['/tmp/track_map.pkl', '/tmp/track_map.json']
        
        for map_file in map_files:
            try:
                if map_file.endswith('.pkl'):
                    with open(map_file, 'rb') as f:
                        map_data = pickle.load(f)
                else:
                    with open(map_file, 'r') as f:
                        map_data = json.load(f)
                
                self.centerline_raw = map_data['centerline']
                
                if len(self.centerline_raw) > 2:
                    self.map_loaded = True
                    rospy.loginfo(f"✅ 成功加载地图: {map_file}")
                    rospy.loginfo(f"📊 中心线点数: {len(self.centerline_raw)}")
                    
                    if 'metadata' in map_data:
                        metadata = map_data['metadata']
                        rospy.loginfo(f"📊 赛道长度: {metadata.get('track_length', 0):.1f}m")
                        rospy.loginfo(f"📊 锥筒总数: {metadata.get('total_cones', 0)}")
                    
                    return
                else:
                    rospy.logwarn(f"地图文件 {map_file} 中中心线点数不足")
                    
            except FileNotFoundError:
                rospy.logwarn(f"地图文件不存在: {map_file}")
            except Exception as e:
                rospy.logwarn(f"加载地图文件失败 {map_file}: {e}")
        
        rospy.logfatal("❌ 所有地图文件加载失败")

    def generate_smooth_path(self):
        """生成平滑路径"""
        if not self.centerline_raw:
            return
        
        try:
            # 提取坐标
            x_points = [p['x'] for p in self.centerline_raw]
            y_points = [p['y'] for p in self.centerline_raw]
            
            # 按X坐标排序
            sorted_indices = np.argsort(x_points)
            x_sorted = np.array([x_points[i] for i in sorted_indices])
            y_sorted = np.array([y_points[i] for i in sorted_indices])
            
            # 去重
            unique_indices = np.where(np.diff(x_sorted) > 0.1)[0] + 1
            if len(unique_indices) > 0:
                unique_indices = np.concatenate(([0], unique_indices))
                x_unique = x_sorted[unique_indices]
                y_unique = y_sorted[unique_indices]
            else:
                x_unique = x_sorted
                y_unique = y_sorted
            
            if len(x_unique) < 3:
                rospy.logwarn("去重后点数不足，使用原始路径")
                self.centerline_smooth = self.centerline_raw.copy()
                return
            
            # B样条平滑
            distances = np.cumsum(np.sqrt(np.diff(x_unique)**2 + np.diff(y_unique)**2))
            t = np.zeros(len(x_unique))
            t[1:] = distances
            
            spline_x = interpolate.UnivariateSpline(t, x_unique, s=self.smoothing_factor, k=min(3, len(t)-1))
            spline_y = interpolate.UnivariateSpline(t, y_unique, s=self.smoothing_factor, k=min(3, len(t)-1))
            
            # 生成平滑路径
            t_smooth = np.linspace(0, t[-1], int(t[-1] / self.path_resolution) + 1)
            x_smooth = spline_x(t_smooth)
            y_smooth = spline_y(t_smooth)
            
            self.centerline_smooth = []
            for i, (x, y) in enumerate(zip(x_smooth, y_smooth)):
                # 🎯 计算朝向角（用于可视化箭头）
                if i < len(x_smooth) - 1:
                    theta = math.atan2(y_smooth[i+1] - y, x_smooth[i+1] - x)
                else:
                    theta = math.atan2(y - y_smooth[i-1], x - x_smooth[i-1])
                
                self.centerline_smooth.append({
                    'x': float(x), 
                    'y': float(y),
                    'theta': theta  # 🎯 添加朝向信息
                })
            
            rospy.loginfo(f"🌊 路径平滑完成: {len(self.centerline_raw)} → {len(self.centerline_smooth)} 点")
            
        except Exception as e:
            rospy.logwarn(f"路径平滑失败，使用原始路径: {e}")
            self.centerline_smooth = self.centerline_raw.copy()

    def publish_planned_path(self, event):
        """发布规划路径"""
        if not self.map_loaded or not self.centerline_smooth:
            # 发布路径不可用信号
            path_available_msg = Bool()
            path_available_msg.data = False
            self.path_available_pub.publish(path_available_msg)
            
            status_msg = String()
            status_msg.data = "❌ 地图未加载或路径不可用"
            self.planner_status_pub.publish(status_msg)
            return
        
        # 发布完整规划路径
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for point in self.centerline_smooth:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            
            # 🎯 添加朝向信息
            if 'theta' in point:
                theta = point['theta']
                pose.pose.orientation.z = math.sin(theta / 2.0)
                pose.pose.orientation.w = math.cos(theta / 2.0)
            else:
                pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.planned_path_pub.publish(path_msg)
        
        # 发布目标速度
        speed_msg = Float32()
        speed_msg.data = self.target_speed
        self.target_speed_pub.publish(speed_msg)
        
        # 发布路径可用信号
        path_available_msg = Bool()
        path_available_msg.data = True
        self.path_available_pub.publish(path_available_msg)
        
        # 发布状态
        status_msg = String()
        status_msg.data = f"✅ 规划路径可用: {len(self.centerline_smooth)}点, 目标速度{self.target_speed}m/s"
        self.planner_status_pub.publish(status_msg)
        
        rospy.loginfo_throttle(10.0, f"📡 发布规划路径: {len(self.centerline_smooth)} 点")

    # 🎨 新增：可视化功能
    def publish_visualizations(self, event):
        """发布可视化标记"""
        if not self.centerline_smooth:
            return
        
        # 发布规划线（绿色线条）
        self.publish_planning_line()
        
        # 发布路径点（绿色球体）
        self.publish_planning_points()
        
        # 发布方向箭头
        self.publish_direction_arrows()

    def publish_planning_line(self):
        """🟢 发布规划线（绿色线条）"""
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "planning_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # 线条属性
        line_marker.scale.x = 0.3  # 线宽
        line_marker.color.r = 0.0  # 绿色
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.9  # 透明度
        
        # 添加路径点
        for point in self.centerline_smooth:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.1  # 稍微抬高避免重叠
            line_marker.points.append(p)
        
        line_marker.lifetime = rospy.Duration(0)  # 永久显示
        self.planning_line_pub.publish(line_marker)

    def publish_planning_points(self):
        """🟢 发布路径点（绿色球体）"""
        marker_array = MarkerArray()
        
        # 每5个点显示一个球体，避免过密
        for i in range(0, len(self.centerline_smooth), 5):
            point = self.centerline_smooth[i]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "planning_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 位置
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            
            # 大小
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # 颜色：亮绿色
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        
        self.planning_points_pub.publish(marker_array)

    def publish_direction_arrows(self):
        """🎯 发布方向箭头"""
        marker_array = MarkerArray()
        
        # 每10个点显示一个箭头
        for i in range(0, len(self.centerline_smooth), 10):
            point = self.centerline_smooth[i]
            
            if 'theta' not in point:
                continue
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "planning_arrows"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # 位置和朝向
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']
            marker.pose.position.z = 0.2
            
            theta = point['theta']
            marker.pose.orientation.z = math.sin(theta / 2.0)
            marker.pose.orientation.w = math.cos(theta / 2.0)
            
            # 箭头大小
            marker.scale.x = 1.0  # 长度
            marker.scale.y = 0.15  # 宽度
            marker.scale.z = 0.15  # 高度
            
            # 颜色：深绿色
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        
        # 合并发布
        all_markers = MarkerArray()
        all_markers.markers.extend(marker_array.markers)
        self.planning_markers_pub.publish(all_markers)

    def calculate_path_length(self):
        """计算路径总长度"""
        if len(self.centerline_smooth) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(self.centerline_smooth) - 1):
            p1 = self.centerline_smooth[i]
            p2 = self.centerline_smooth[i + 1]
            length = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
            total_length += length
        
        return total_length

if __name__ == '__main__':
    try:
        planner = MapBasedPlanner()
        rospy.loginfo("🛣️ 基于地图的路径规划器运行中...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛣️ 路径规划器关闭")