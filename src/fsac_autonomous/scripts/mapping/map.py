#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
import pickle  # 新增
import json    # 新增
from fsd_common_msgs.msg import ConeDetections, CarState
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String  # 新增

class DynamicMapper:
    def __init__(self):
        rospy.init_node('dynamic_mapper')  # 移到这里
        
        # 订阅话题
        self.cone_sub = rospy.Subscriber('/perception/lidar/cone_detections', ConeDetections, self.cone_callback)
        self.state_sub = rospy.Subscriber('/estimation/slam/state', CarState, self.state_callback)
        
        # 发布话题
        self.marker_pub = rospy.Publisher('/mapping/cone_markers', MarkerArray, queue_size=10)
        self.track_boundary_pub = rospy.Publisher('/mapping/track_boundaries', MarkerArray, queue_size=10)
        self.centerline_pub = rospy.Publisher('/mapping/centerline', Path, queue_size=10)
        self.centerline_marker_pub = rospy.Publisher('/mapping/centerline_markers', MarkerArray, queue_size=10)
        
        # 🎨 新增：额外的可视化发布器
        self.boundary_markers_pub = rospy.Publisher('/mapping/boundary_markers', MarkerArray, queue_size=10)
        self.car_trajectory_pub = rospy.Publisher('/mapping/car_trajectory', Path, queue_size=10)
        self.car_trajectory_markers_pub = rospy.Publisher('/mapping/car_trajectory_markers', MarkerArray, queue_size=10)
        self.status_pub = rospy.Publisher('/mapping/status', String, queue_size=10)
        
        # 存储车辆状态
        self.car_state = None
        
        # 全局锥筒存储 - 分左右存储
        self.left_cones_global = []   # 全局左侧锥筒
        self.right_cones_global = []  # 全局右侧锥筒
        
        # 赛道信息
        self.left_boundary = []   # 左边界点
        self.right_boundary = []  # 右边界点
        self.centerline = []      # 中心线点
        
        # 🚗 新增：车辆轨迹记录
        self.car_trajectory = []  # 记录车辆轨迹
        
        # 记录初始车辆位置作为原点
        self.origin_x = None
        self.origin_y = None
        
        # 过滤参数
        self.max_detection_distance = 50.0  # 最大检测距离（米）
        self.min_detection_distance = 1.0   # 最小检测距离（米）
        self.max_track_width = 8.0          # 最大赛道宽度（米）
        self.min_track_width = 2.0          # 最小赛道宽度（米）
        self.outlier_distance_threshold = 5.0  # 异常点距离阈值（米）
        
        # 定时器：定期更新赛道信息
        self.track_timer = rospy.Timer(rospy.Duration(2.0), self.update_track_info)
        
        # 🎨 新增：可视化和保存定时器
        self.viz_timer = rospy.Timer(rospy.Duration(1.0), self.publish_all_visualizations)
        self.save_timer = rospy.Timer(rospy.Duration(10.0), self.save_map_to_file)
        
        rospy.loginfo("🚀 改进的动态建图节点启动 - 雷达坐标系分类 + 多重过滤 + 完整可视化")

    def state_callback(self, msg):
        """更新车辆状态"""
        try:
            if self.origin_x is None:
                self.origin_x = msg.car_state.x
                self.origin_y = msg.car_state.y
                rospy.loginfo(f"设置原点: ({self.origin_x:.2f}, {self.origin_y:.2f})")
            
            self.car_state = {
                'x': msg.car_state.x - self.origin_x,
                'y': msg.car_state.y - self.origin_y,
                'theta': msg.car_state.theta
            }
            
            # 🚗 记录车辆轨迹
            self.car_trajectory.append({
                'x': self.car_state['x'],
                'y': self.car_state['y'],
                'theta': self.car_state['theta'],
                'timestamp': rospy.Time.now().to_sec()
            })
            
            # 限制轨迹点数量，避免过多
            if len(self.car_trajectory) > 1000:
                self.car_trajectory = self.car_trajectory[-800:]  # 保留最近800个点
            
            rospy.loginfo_throttle(5.0, f"车辆相对位置: x={self.car_state['x']:.2f}, y={self.car_state['y']:.2f}")
            
        except AttributeError as e:
            rospy.logerr(f"车辆状态数据访问错误: {e}")

    def cone_callback(self, msg):
        """处理锥筒检测 - 改进版"""
        if not self.car_state:
            rospy.logwarn("车辆状态未初始化，无法进行坐标转换")
            return
        
        # 步骤1：过滤置信度
        high_confidence_cones = []
        for cone in msg.cone_detections:
            if cone.poseConfidence.data == 1.0:
                high_confidence_cones.append(cone)
        
        if not high_confidence_cones:
            rospy.logwarn("没有置信度为1.0的锥筒")
            return
            
        rospy.loginfo(f"=== 收到 {len(msg.cone_detections)} 个锥筒，其中 {len(high_confidence_cones)} 个置信度为1.0 ===")
        
        # 步骤2：在雷达坐标系中进行初步分类和过滤
        left_cones_lidar, right_cones_lidar = self.classify_and_filter_in_lidar_frame(high_confidence_cones)
        
        if not left_cones_lidar and not right_cones_lidar:
            rospy.logwarn("经过过滤后没有有效锥筒")
            return
        
        rospy.loginfo(f"雷达坐标系分类结果: 左侧 {len(left_cones_lidar)} 个, 右侧 {len(right_cones_lidar)} 个")
        
        # 步骤3：转换到世界坐标系并添加到全局地图
        new_left_count = 0
        new_right_count = 0
        
        for cone in left_cones_lidar:
            world_cone = self.transform_cone_to_world(cone)
            if self.is_unique_cone(world_cone, self.left_cones_global):
                self.left_cones_global.append(world_cone)
                new_left_count += 1
        
        for cone in right_cones_lidar:
            world_cone = self.transform_cone_to_world(cone)
            if self.is_unique_cone(world_cone, self.right_cones_global):
                self.right_cones_global.append(world_cone)
                new_right_count += 1
        
        # 步骤4：全局异常点过滤
        self.filter_global_outliers()
        
        # 步骤5：发布标记
        all_cones = self.left_cones_global + self.right_cones_global
        if all_cones:
            self.publish_markers_with_classification()
            rospy.loginfo(f"📍 本次添加: 左侧 {new_left_count} 个, 右侧 {new_right_count} 个")
            rospy.loginfo(f"📍 全局总数: 左侧 {len(self.left_cones_global)} 个, 右侧 {len(self.right_cones_global)} 个")

    def classify_and_filter_in_lidar_frame(self, cones):
        """在雷达坐标系中分类和过滤锥筒"""
        left_cones = []
        right_cones = []
        
        for i, cone in enumerate(cones):
            # 距离过滤
            distance = math.sqrt(cone.position.x**2 + cone.position.y**2)
            if distance < self.min_detection_distance or distance > self.max_detection_distance:
                rospy.loginfo(f"锥筒 {i}: 距离 {distance:.2f}m 超出范围 [{self.min_detection_distance}, {self.max_detection_distance}]")
                continue
            
            # 角度过滤：只保留前方±120度范围内的锥筒
            angle = math.atan2(cone.position.y, cone.position.x)
            if abs(angle) > math.radians(120):
                rospy.loginfo(f"锥筒 {i}: 角度 {math.degrees(angle):.1f}° 超出前方范围")
                continue
            
            # 位置过滤：排除车辆后方的锥筒
            if cone.position.x < -1.0:  # 车辆后方1米以外的锥筒
                rospy.loginfo(f"锥筒 {i}: X={cone.position.x:.2f} 位于车辆后方")
                continue
            
            # 左右分类：在雷达坐标系中，Y>0为左侧，Y<0为右侧
            if cone.position.y > 0:
                left_cones.append(cone)
                rospy.loginfo(f"✅ 左侧锥筒 {i}: 位置({cone.position.x:.2f}, {cone.position.y:.2f}), 距离{distance:.2f}m")
            else:
                right_cones.append(cone)
                rospy.loginfo(f"✅ 右侧锥筒 {i}: 位置({cone.position.x:.2f}, {cone.position.y:.2f}), 距离{distance:.2f}m")
        
        # 宽度合理性检查
        left_cones, right_cones = self.filter_by_track_width(left_cones, right_cones)
        
        return left_cones, right_cones

    def filter_by_track_width(self, left_cones, right_cones):
        """基于赛道宽度过滤锥筒"""
        filtered_left = []
        filtered_right = []
        
        for left_cone in left_cones:
            min_distance_to_right = float('inf')
            
            # 找到最近的右侧锥筒
            for right_cone in right_cones:
                # 计算相同X坐标范围内的距离
                if abs(left_cone.position.x - right_cone.position.x) < 5.0:  # X坐标差小于5米
                    distance = abs(left_cone.position.y - right_cone.position.y)
                    min_distance_to_right = min(min_distance_to_right, distance)
            
            # 检查赛道宽度是否合理
            if min_distance_to_right != float('inf'):
                if self.min_track_width <= min_distance_to_right <= self.max_track_width:
                    filtered_left.append(left_cone)
                else:
                    rospy.loginfo(f"❌ 左侧锥筒过滤: 与右侧距离 {min_distance_to_right:.2f}m 不在合理范围 [{self.min_track_width}, {self.max_track_width}]")
            else:
                # 如果没有对应的右侧锥筒，但距离不太远，也保留
                if left_cone.position.y < self.max_track_width:
                    filtered_left.append(left_cone)
        
        # 对右侧锥筒进行相同处理
        for right_cone in right_cones:
            min_distance_to_left = float('inf')
            
            for left_cone in left_cones:
                if abs(left_cone.position.x - right_cone.position.x) < 5.0:
                    distance = abs(left_cone.position.y - right_cone.position.y)
                    min_distance_to_left = min(min_distance_to_left, distance)
            
            if min_distance_to_left != float('inf'):
                if self.min_track_width <= min_distance_to_left <= self.max_track_width:
                    filtered_right.append(right_cone)
                else:
                    rospy.loginfo(f"❌ 右侧锥筒过滤: 与左侧距离 {min_distance_to_left:.2f}m 不在合理范围")
            else:
                if abs(right_cone.position.y) < self.max_track_width:
                    filtered_right.append(right_cone)
        
        return filtered_left, filtered_right

    def filter_global_outliers(self):
        """过滤全局异常点"""
        # 对左侧锥筒进行异常点检测
        self.left_cones_global = self.remove_outliers(self.left_cones_global)
        # 对右侧锥筒进行异常点检测  
        self.right_cones_global = self.remove_outliers(self.right_cones_global)

    def remove_outliers(self, cones):
        """移除异常点"""
        if len(cones) < 3:
            return cones
        
        filtered_cones = []
        
        for i, cone in enumerate(cones):
            # 计算与其他锥筒的平均距离
            distances = []
            for j, other_cone in enumerate(cones):
                if i != j:
                    dist = math.sqrt((cone['x'] - other_cone['x'])**2 + (cone['y'] - other_cone['y'])**2)
                    distances.append(dist)
            
            if distances:
                min_distance = min(distances)
                # 如果与最近锥筒的距离过大，认为是异常点
                if min_distance <= self.outlier_distance_threshold:
                    filtered_cones.append(cone)
                else:
                    rospy.loginfo(f"🗑️ 移除异常锥筒: 位置({cone['x']:.2f}, {cone['y']:.2f}), 最近距离{min_distance:.2f}m")
        
        return filtered_cones

    def transform_cone_to_world(self, cone):
        """坐标转换"""
        car_x = self.car_state['x']
        car_y = self.car_state['y']
        car_theta = self.car_state['theta']
        
        cone_x_vehicle = cone.position.x - 2.4
        cone_y_vehicle = cone.position.y
        
        world_x = car_x + cone_x_vehicle * math.cos(car_theta) - cone_y_vehicle * math.sin(car_theta)
        world_y = car_y + cone_x_vehicle * math.sin(car_theta) + cone_y_vehicle * math.cos(car_theta)
        
        return {'x': world_x, 'y': world_y, 'color': cone.color.data}

    def is_unique_cone(self, new_cone, existing_cones):
        """检查锥筒是否唯一"""
        threshold = 0.8
        for cone in existing_cones:
            distance = math.sqrt((new_cone['x'] - cone['x'])**2 + (new_cone['y'] - cone['y'])**2)
            if distance < threshold:
                return False
        return True

    def update_track_info(self, event):
        """定期更新赛道信息"""
        total_cones = len(self.left_cones_global) + len(self.right_cones_global)
        if total_cones < 4:
            rospy.loginfo_throttle(10.0, f"锥筒数量不足，当前 {total_cones} 个，需要至少 4 个")
            return
        
        self.generate_track_boundaries()
        self.generate_centerline()
        
        self.publish_track_boundaries()
        self.publish_centerline()

    def generate_track_boundaries(self):
        """生成赛道边界 - 基于已分类的锥筒"""
        # 按X坐标排序
        self.left_boundary = sorted(self.left_cones_global, key=lambda c: c['x'])
        self.right_boundary = sorted(self.right_cones_global, key=lambda c: c['x'])
        
        rospy.loginfo(f"生成赛道边界: 左边界 {len(self.left_boundary)} 点, 右边界 {len(self.right_boundary)} 点")

    def generate_centerline(self):
        """基于边界生成中心线"""
        if len(self.left_boundary) < 2 or len(self.right_boundary) < 2:
            rospy.logwarn(f"边界点不足: 左边界{len(self.left_boundary)}点, 右边界{len(self.right_boundary)}点")
            return
        
        self.centerline = []
        
        # 找到X坐标的重叠范围
        left_x_coords = [cone['x'] for cone in self.left_boundary]
        right_x_coords = [cone['x'] for cone in self.right_boundary]
        
        x_start = max(min(left_x_coords), min(right_x_coords))
        x_end = min(max(left_x_coords), max(right_x_coords))
        
        if x_start >= x_end:
            rospy.logwarn(f"边界范围不重叠: 左边界X=[{min(left_x_coords):.2f}, {max(left_x_coords):.2f}], 右边界X=[{min(right_x_coords):.2f}, {max(right_x_coords):.2f}]")
            return
        
        # 在重叠范围内生成中心线点
        num_points = max(10, int((x_end - x_start) / 0.5) + 1)
        
        for i in range(num_points):
            if num_points == 1:
                x = x_start
            else:
                x = x_start + i * (x_end - x_start) / (num_points - 1)
            
            left_y = self.interpolate_y(self.left_boundary, x)
            right_y = self.interpolate_y(self.right_boundary, x)
            
            if left_y is not None and right_y is not None:
                center_y = (left_y + right_y) / 2
                self.centerline.append({'x': x, 'y': center_y})
        
        rospy.loginfo(f"生成中心线: {len(self.centerline)} 个点，范围 X=[{x_start:.2f}, {x_end:.2f}]")

    def interpolate_y(self, boundary, x):
        """在边界上插值得到指定X坐标的Y值"""
        if len(boundary) < 2:
            return None
        
        # 找到X坐标两侧的点
        for i in range(len(boundary) - 1):
            if boundary[i]['x'] <= x <= boundary[i+1]['x']:
                x1, y1 = boundary[i]['x'], boundary[i]['y']
                x2, y2 = boundary[i+1]['x'], boundary[i+1]['y']
                
                if x2 == x1:
                    return y1
                
                y = y1 + (y2 - y1) * (x - x1) / (x2 - x1)
                return y
        
        # 如果x超出范围，使用最近的边界点
        if x < boundary[0]['x']:
            return boundary[0]['y']
        elif x > boundary[-1]['x']:
            return boundary[-1]['y']
        
        return None

    # 🎨 新增：完整的可视化功能
    def publish_all_visualizations(self, event):
        """发布所有可视化内容"""
        self.publish_markers_with_classification()
        self.publish_enhanced_boundary_markers()
        self.publish_car_trajectory_visualization()
        self.publish_status_info()

    def publish_enhanced_boundary_markers(self):
        """📏 发布增强的边界线标记"""
        if len(self.left_boundary) < 2 and len(self.right_boundary) < 2:
            return
        
        marker_array = MarkerArray()
        
        # 左边界线（粗蓝线）
        if len(self.left_boundary) > 1:
            left_marker = self.create_boundary_line_marker(
                self.left_boundary, 0, "enhanced_left_boundary", 
                (0.0, 0.0, 1.0, 0.8), 0.4
            )
            marker_array.markers.append(left_marker)
        
        # 右边界线（粗红线）
        if len(self.right_boundary) > 1:
            right_marker = self.create_boundary_line_marker(
                self.right_boundary, 1, "enhanced_right_boundary", 
                (1.0, 0.0, 0.0, 0.8), 0.4
            )
            marker_array.markers.append(right_marker)
        
        self.boundary_markers_pub.publish(marker_array)

    def publish_car_trajectory_visualization(self):
        """🚗 发布车辆轨迹可视化"""
        if len(self.car_trajectory) < 2:
            return
        
        # 发布轨迹路径
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        # 每10个点取一个，避免轨迹过密
        for point in self.car_trajectory[::10]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            
            # 设置朝向
            theta = point['theta']
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            
            path_msg.poses.append(pose)
        
        self.car_trajectory_pub.publish(path_msg)
        
        # 发布轨迹标记
        marker_array = MarkerArray()
        
        # 轨迹线
        trajectory_marker = self.create_trajectory_line_marker(
            self.car_trajectory, 0, "car_trajectory", (1.0, 1.0, 0.0, 0.7)
        )
        marker_array.markers.append(trajectory_marker)
        
        # 当前车辆位置
        if self.car_trajectory:
            current_pos = self.car_trajectory[-1]
            car_marker = self.create_car_marker(
                current_pos, 1, "current_car", (1.0, 0.5, 0.0, 1.0)
            )
            marker_array.markers.append(car_marker)
        
        self.car_trajectory_markers_pub.publish(marker_array)

    def publish_status_info(self):
        """📊 发布状态信息"""
        total_distance = 0.0
        if len(self.car_trajectory) > 1:
            for i in range(len(self.car_trajectory) - 1):
                p1 = self.car_trajectory[i]
                p2 = self.car_trajectory[i + 1]
                dist = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
                total_distance += dist
        
        track_length = self.calculate_track_length()
        
        status = f"🗺️ 建图状态: 锥筒{len(self.left_cones_global)+len(self.right_cones_global)}个 | " \
                f"中心线{len(self.centerline)}点({track_length:.1f}m) | " \
                f"行驶{total_distance:.1f}m | " \
                f"轨迹{len(self.car_trajectory)}点"
        
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    # 🎨 标记创建辅助方法
    def create_boundary_line_marker(self, boundary_points, marker_id, ns, color, width):
        """创建边界线标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        for point in boundary_points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.1
            marker.points.append(p)
        
        marker.lifetime = rospy.Duration(0)
        return marker

    def create_trajectory_line_marker(self, trajectory_points, marker_id, ns, color):
        """创建轨迹线标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.2  # 轨迹线宽度
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # 每5个点取一个，避免过密
        for point in trajectory_points[::5]:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.05
            marker.points.append(p)
        
        marker.lifetime = rospy.Duration(0)
        return marker

    def create_car_marker(self, car_pos, marker_id, ns, color):
        """创建车辆标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = car_pos['x']
        marker.pose.position.y = car_pos['y']
        marker.pose.position.z = 0.5
        
        # 设置朝向
        theta = car_pos['theta']
        marker.pose.orientation.z = math.sin(theta / 2.0)
        marker.pose.orientation.w = math.cos(theta / 2.0)
        
        marker.scale.x = 2.0  # 长度
        marker.scale.y = 0.5  # 宽度
        marker.scale.z = 0.3  # 高度
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker.lifetime = rospy.Duration(0)
        return marker

    # 💾 新增：地图保存功能
    def save_map_to_file(self, event):
        """保存地图到文件"""
        if len(self.left_cones_global) + len(self.right_cones_global) < 10:
            return
        
        map_data = {
            'left_cones': self.left_cones_global,
            'right_cones': self.right_cones_global,
            'centerline': self.centerline,
            'car_trajectory': self.car_trajectory[-100:],  # 只保存最后100个点
            'metadata': {
                'total_cones': len(self.left_cones_global) + len(self.right_cones_global),
                'centerline_length': len(self.centerline),
                'track_length': self.calculate_track_length(),
                'origin_x': self.origin_x,
                'origin_y': self.origin_y
            }
        }
        
        # 保存为JSON文件（便于查看）
        try:
            with open('/tmp/track_map.json', 'w') as f:
                json.dump(map_data, f, indent=2)
            rospy.loginfo_throttle(30.0, "💾 地图已保存到 /tmp/track_map.json")
        except Exception as e:
            rospy.logwarn(f"保存JSON失败: {e}")
        
        # 保存为pickle文件（便于程序读取）
        try:
            with open('/tmp/track_map.pkl', 'wb') as f:
                pickle.dump(map_data, f)
            rospy.loginfo_throttle(30.0, "💾 地图已保存到 /tmp/track_map.pkl")
        except Exception as e:
            rospy.logwarn(f"保存pickle失败: {e}")

    def calculate_track_length(self):
        """计算赛道长度"""
        if len(self.centerline) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(self.centerline) - 1):
            p1 = self.centerline[i]
            p2 = self.centerline[i + 1]
            length = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
            total_length += length
        
        return total_length

    # 保持原有的方法不变
    def publish_markers_with_classification(self):
        """发布已分类的锥筒标记"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # 发布左侧锥筒（蓝色）
        for cone in self.left_cones_global:
            marker = self.create_cone_marker(cone, marker_id, "left", (0.0, 0.0, 1.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        # 发布右侧锥筒（红色）
        for cone in self.right_cones_global:
            marker = self.create_cone_marker(cone, marker_id, "right", (1.0, 0.0, 0.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.marker_pub.publish(marker_array)
        rospy.loginfo_throttle(5.0, f"🎯 发布锥筒标记: 左侧{len(self.left_cones_global)}个(蓝色), 右侧{len(self.right_cones_global)}个(红色)")

    def create_cone_marker(self, cone, marker_id, side, color):
        """创建锥筒标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"cones_{side}"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = cone['x']
        marker.pose.position.y = cone['y']
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 1.0
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(0)
        return marker

    def publish_track_boundaries(self):
        """发布赛道边界"""
        if len(self.left_boundary) < 2 and len(self.right_boundary) < 2:
            return
        
        marker_array = MarkerArray()
        
        # 左边界线
        if len(self.left_boundary) > 1:
            left_marker = Marker()
            left_marker.header.frame_id = "map"
            left_marker.header.stamp = rospy.Time.now()
            left_marker.ns = "boundaries"
            left_marker.id = 0
            left_marker.type = Marker.LINE_STRIP
            left_marker.action = Marker.ADD
            left_marker.scale.x = 0.3
            left_marker.color.r = 0.0
            left_marker.color.g = 0.0
            left_marker.color.b = 1.0
            left_marker.color.a = 1.0
            
            for point in self.left_boundary:
                p = Point()
                p.x = point['x']
                p.y = point['y']
                p.z = 0.1
                left_marker.points.append(p)
            
            marker_array.markers.append(left_marker)
        
        # 右边界线
        if len(self.right_boundary) > 1:
            right_marker = Marker()
            right_marker.header.frame_id = "map"
            right_marker.header.stamp = rospy.Time.now()
            right_marker.ns = "boundaries"
            right_marker.id = 1
            right_marker.type = Marker.LINE_STRIP
            right_marker.action = Marker.ADD
            right_marker.scale.x = 0.3
            right_marker.color.r = 1.0
            right_marker.color.g = 0.0
            right_marker.color.b = 0.0
            right_marker.color.a = 1.0
            
            for point in self.right_boundary:
                p = Point()
                p.x = point['x']
                p.y = point['y']
                p.z = 0.1
                right_marker.points.append(p)
            
            marker_array.markers.append(right_marker)
        
        self.track_boundary_pub.publish(marker_array)

    def publish_centerline(self):
        """发布中心线"""
        if len(self.centerline) < 2:
            return
        
        # 发布为 Path 消息
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for point in self.centerline:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.centerline_pub.publish(path_msg)
        
        # 发布可视化标记
        marker_array = MarkerArray()
        centerline_marker = Marker()
        centerline_marker.header.frame_id = "map"
        centerline_marker.header.stamp = rospy.Time.now()
        centerline_marker.ns = "centerline"
        centerline_marker.id = 0
        centerline_marker.type = Marker.LINE_STRIP
        centerline_marker.action = Marker.ADD
        centerline_marker.scale.x = 0.4
        centerline_marker.color.r = 0.0
        centerline_marker.color.g = 1.0
        centerline_marker.color.b = 0.0
        centerline_marker.color.a = 1.0
        
        for point in self.centerline:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.2
            centerline_marker.points.append(p)
        
        marker_array.markers.append(centerline_marker)
        self.centerline_marker_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.loginfo("🚀 改进的动态建图节点启动 - 雷达坐标系分类 + 多重过滤 + 完整可视化")
    mapper = DynamicMapper()
    rospy.spin()