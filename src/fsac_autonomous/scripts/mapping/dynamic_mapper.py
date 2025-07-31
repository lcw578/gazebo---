#!/usr/bin/env python3
# filepath: [dynamic_mapper.py](http://_vscodecontentref_/5)
# -*- coding: utf-8 -*-
"""
动态建图模块
功能：实时建图，构建赛道地图，不允许完全建图后再规划
作者：[你的姓名]
日期：[日期]
"""
import rospy
import math
import numpy as np
from fsd_common_msgs.msg import ConeDetections, CarState
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Header
import tf2_ros

class DynamicMapper:
    def __init__(self):
        rospy.init_node('dynamic_mapper', anonymous=True)
        
        # =====================================================
        # 赛道参数（根据题目要求）
        # =====================================================
        self.track_length = 75.0        # 加速段长度 75米
        self.decel_length = 100.0       # 减速段长度 100米  
        self.track_width = 4.9          # 最小赛道宽度 4.9米
        self.cone_interval = 5.0        # 锥筒间隔 5米
        self.start_offset = 0.3         # 起点偏移 0.3米
        
        # =====================================================
        # 建图参数
        # =====================================================
        self.map_resolution = 0.1       # 地图分辨率 10cm
        self.map_width = 200            # 地图宽度（米）
        self.map_height = 20            # 地图高度（米）
        self.lidar_offset_x = 2.4       # 激光雷达X偏移
        
        # 锥筒分类参数
        self.cone_confidence_threshold = 0.3
        self.duplicate_threshold = 0.8   # 重复锥筒判断阈值
        
        # =====================================================
        # 数据存储
        # =====================================================
        self.left_red_cones = []        # 左侧红色锥筒（加速段）
        self.right_blue_cones = []      # 右侧蓝色锥筒（加速段）
        self.yellow_cones = []          # 黄色锥筒（减速段）
        self.all_cones = []             # 所有锥筒
        
        self.current_car_state = None
        self.track_boundaries = {
            'left': [],                 # 左边界点
            'right': [],                # 右边界点
            'centerline': []            # 中心线点
        }
        
        # 建图状态
        self.mapping_progress = 0.0     # 建图进度 0-1
        self.track_sections = {
            'acceleration': False,       # 是否检测到加速段
            'deceleration': False       # 是否检测到减速段
        }
        
        # =====================================================
        # TF系统
        # =====================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # =====================================================
        # 订阅器
        # =====================================================
        self.cone_sub = rospy.Subscriber(
            '/perception/lidar/cone_detections',
            ConeDetections,
            self.cone_detection_callback,  # 统一使用这个回调函数
            queue_size=50
        )
        
        self.state_sub = rospy.Subscriber(
            '/estimation/slam/state',
            CarState,
            self.car_state_callback,
            queue_size=50
        )
        
        # =====================================================
        # 发布器
        # =====================================================
        # 建图结果
        self.map_pub = rospy.Publisher('/mapping/track_map', MarkerArray, queue_size=10)
        self.boundary_pub = rospy.Publisher('/mapping/track_boundaries', MarkerArray, queue_size=10)
        self.occupancy_pub = rospy.Publisher('/mapping/occupancy_grid', OccupancyGrid, queue_size=10)
        
        # 锥筒分类发布
        self.red_cones_pub = rospy.Publisher('/mapping/red_cones', MarkerArray, queue_size=10)
        self.blue_cones_pub = rospy.Publisher('/mapping/blue_cones', MarkerArray, queue_size=10)
        self.yellow_cones_pub = rospy.Publisher('/mapping/yellow_cones', MarkerArray, queue_size=10)
        
        # 建图状态
        self.mapping_status_pub = rospy.Publisher('/mapping/status', String, queue_size=10)
        
        # =====================================================
        # 定时器
        # =====================================================
        self.mapping_timer = rospy.Timer(rospy.Duration(0.2), self.update_map)      # 5Hz建图更新
        self.viz_timer = rospy.Timer(rospy.Duration(0.5), self.publish_visualization) # 2Hz可视化
        
        rospy.loginfo("=== 动态建图模块启动 ===")
        rospy.loginfo(f"赛道参数: 加速段{self.track_length}m, 减速段{self.decel_length}m, 宽度≥{self.track_width}m")
        rospy.loginfo(f"锥筒布局: 前75m左红右蓝, 后100m全黄, 间隔{self.cone_interval}m")

    def cone_detection_callback(self, msg):
        """
        锥筒检测回调函数（主要回调函数）
        实时处理检测到的锥筒，进行分类和存储
        """
        if self.current_car_state is None:
            rospy.logdebug("等待车辆状态数据...")
            return
        
        new_cones_count = 0
        processed_cones = 0
        
        # 修改这里：使用正确的字段名 cone_detections
        for cone in msg.cone_detections:
            processed_cones += 1
            
            # 置信度检查
            if (cone.poseConfidence.data < self.cone_confidence_threshold or 
                cone.colorConfidence.data < self.cone_confidence_threshold):
                continue
            
            # 坐标变换到世界坐标系
            world_cone = self.transform_cone_to_world(cone)
            if world_cone is None:
                continue
            
            # 检查重复
            if self.is_duplicate_cone(world_cone):
                continue
            
            # 添加到总列表
            self.all_cones.append(world_cone)
            
            # 按颜色和位置分类
            self.classify_and_store_cone(world_cone)
            
            new_cones_count += 1
            
            rospy.logdebug(f"新增{self.get_color_name(world_cone['color'])}锥筒: "
                          f"({world_cone['x']:.2f}, {world_cone['y']:.2f})")
        
        if new_cones_count > 0:
            rospy.loginfo(f"处理 {processed_cones} 个检测，新增 {new_cones_count} 个锥筒")
            rospy.loginfo(f"当前地图: 红色{len(self.left_red_cones)}, "
                         f"蓝色{len(self.right_blue_cones)}, 黄色{len(self.yellow_cones)}")

    def car_state_callback(self, msg):
        """
        车辆状态回调函数
        更新当前车辆位置，用于坐标变换和建图进度计算
        """
        self.current_car_state = {
            'x': msg.car_state.x,
            'y': msg.car_state.y,
            'theta': msg.car_state.theta,
            'vx': msg.car_state_dt.car_state_dt.x,
            'vy': msg.car_state_dt.car_state_dt.y,
            'vtheta': msg.car_state_dt.car_state_dt.theta
        }
        
        # 计算建图进度（基于车辆X位置）
        progress = max(0.0, min(1.0, (self.current_car_state['x'] + self.start_offset) / self.track_length))
        self.mapping_progress = progress

    def transform_cone_to_world(self, cone):
        """
        坐标变换：激光雷达坐标系 -> 世界坐标系
        考虑激光雷达安装偏移和车辆当前姿态
        """
        try:
            # 激光雷达检测到的锥筒坐标
            cone_x_lidar = cone.position.x
            cone_y_lidar = cone.position.y
            cone_z_lidar = cone.position.z
            
            # 当前车辆状态
            car_x = self.current_car_state['x']
            car_y = self.current_car_state['y']
            car_theta = self.current_car_state['theta']
            
            # 步骤1: 激光雷达坐标系 -> 车辆坐标系（考虑安装偏移）
            cone_x_vehicle = cone_x_lidar + self.lidar_offset_x
            cone_y_vehicle = cone_y_lidar
            
            # 步骤2: 车辆坐标系 -> 世界坐标系（旋转+平移变换）
            cos_theta = math.cos(car_theta)
            sin_theta = math.sin(car_theta)
            
            world_x = car_x + cone_x_vehicle * cos_theta - cone_y_vehicle * sin_theta
            world_y = car_y + cone_x_vehicle * sin_theta + cone_y_vehicle * cos_theta
            
            # 获取颜色 - 适配字符串类型
            color = cone.color.data if hasattr(cone.color, 'data') else str(cone.color)
            
            return {
                'x': world_x,
                'y': world_y,
                'z': cone_z_lidar,
                'color': color,
                'pose_confidence': cone.poseConfidence.data,
                'color_confidence': cone.colorConfidence.data,
                'detection_time': rospy.Time.now(),
                'car_position_when_detected': (car_x, car_y, car_theta)
            }
            
        except Exception as e:
            rospy.logwarn(f"坐标变换失败: {e}")
            return None

    def is_duplicate_cone(self, new_cone):
        """
        检查是否为重复锥筒
        基于欧几里得距离判断
        """
        for existing_cone in self.all_cones:
            distance = math.sqrt(
                (new_cone['x'] - existing_cone['x'])**2 + 
                (new_cone['y'] - existing_cone['y'])**2
            )
            if distance < self.duplicate_threshold:
                return True
        return False

    def classify_and_store_cone(self, cone):
        """
        根据颜色和位置对锥筒进行分类存储
        按照赛道规则：前75米左红右蓝，后100米全黄
        """
        color = cone['color'].lower().strip()
        cone_x = cone['x']
        
        # 判断锥筒所在赛道段
        if cone_x <= self.track_length:  # 加速段 (0-75米)
            if color in ['r', 'red', '红']:
                self.left_red_cones.append(cone)
                self.track_sections['acceleration'] = True
                rospy.logdebug(f"加速段左侧红锥筒: x={cone_x:.2f}")
                
            elif color in ['b', 'blue', '蓝']:
                self.right_blue_cones.append(cone)
                self.track_sections['acceleration'] = True
                rospy.logdebug(f"加速段右侧蓝锥筒: x={cone_x:.2f}")
                
        elif cone_x > self.track_length:  # 减速段 (75-175米)
            if color in ['y', 'yellow', '黄']:
                self.yellow_cones.append(cone)
                self.track_sections['deceleration'] = True
                rospy.logdebug(f"减速段黄锥筒: x={cone_x:.2f}")
        
        # 如果颜色不符合预期，记录警告
        else:
            rospy.logwarn(f"意外的锥筒颜色或位置: {color} at x={cone_x:.2f}")

    def update_map(self, event):
        """
        更新地图信息
        实时计算赛道边界和中心线
        """
        if len(self.all_cones) < 2:
            return
        
        # 更新赛道边界
        self.update_track_boundaries()
        
        # 计算中心线（动态规划的基础）
        self.calculate_dynamic_centerline()
        
        # 检查建图完整性
        self.check_mapping_completeness()

    def update_track_boundaries(self):
        """
        基于检测到的锥筒更新赛道边界
        分段处理：加速段（红蓝锥筒）和减速段（黄锥筒）
        """
        # 清空之前的边界
        self.track_boundaries['left'].clear()
        self.track_boundaries['right'].clear()
        
        # 加速段边界（0-75米）
        if self.left_red_cones:
            # 按X坐标排序红锥筒
            sorted_red = sorted(self.left_red_cones, key=lambda c: c['x'])
            self.track_boundaries['left'].extend([(c['x'], c['y']) for c in sorted_red])
        
        if self.right_blue_cones:
            # 按X坐标排序蓝锥筒
            sorted_blue = sorted(self.right_blue_cones, key=lambda c: c['x'])
            self.track_boundaries['right'].extend([(c['x'], c['y']) for c in sorted_blue])
        
        # 减速段边界（75-175米）
        if self.yellow_cones:
            # 将黄锥筒分为左右两侧
            left_yellow, right_yellow = self.separate_yellow_cones()
            
            # 按X坐标排序并添加到边界
            if left_yellow:
                sorted_left_yellow = sorted(left_yellow, key=lambda c: c['x'])
                self.track_boundaries['left'].extend([(c['x'], c['y']) for c in sorted_left_yellow])
            
            if right_yellow:
                sorted_right_yellow = sorted(right_yellow, key=lambda c: c['x'])
                self.track_boundaries['right'].extend([(c['x'], c['y']) for c in sorted_right_yellow])

    def separate_yellow_cones(self):
        """
        将减速段的黄锥筒分为左右两侧
        基于Y坐标和赛道中心线判断
        """
        if not self.yellow_cones:
            return [], []
        
        # 估计赛道中心Y坐标（基于已知的红蓝锥筒）
        center_y = 0.0
        if self.left_red_cones and self.right_blue_cones:
            avg_red_y = np.mean([c['y'] for c in self.left_red_cones])
            avg_blue_y = np.mean([c['y'] for c in self.right_blue_cones])
            center_y = (avg_red_y + avg_blue_y) / 2.0
        
        left_yellow = []
        right_yellow = []
        
        for cone in self.yellow_cones:
            if cone['y'] > center_y:
                left_yellow.append(cone)  # Y坐标较大的认为是左侧
            else:
                right_yellow.append(cone)  # Y坐标较小的认为是右侧
        
        return left_yellow, right_yellow

    def calculate_dynamic_centerline(self):
        """
        动态计算中心线
        这是路径规划的基础，必须实时更新
        """
        self.track_boundaries['centerline'].clear()
        
        if (len(self.track_boundaries['left']) < 2 or 
            len(self.track_boundaries['right']) < 2):
            return
        
        # 获取左右边界点
        left_points = self.track_boundaries['left']
        right_points = self.track_boundaries['right']
        
        # 找到共同的X范围
        left_x_range = [p[0] for p in left_points]
        right_x_range = [p[0] for p in right_points]
        
        min_x = max(min(left_x_range), min(right_x_range))
        max_x = min(max(left_x_range), max(right_x_range))
        
        # 在共同X范围内插值计算中心线
        x_step = 1.0  # 每1米一个中心线点
        current_x = min_x
        
        while current_x <= max_x:
            left_y = self.interpolate_y_at_x(left_points, current_x)
            right_y = self.interpolate_y_at_x(right_points, current_x)
            
            if left_y is not None and right_y is not None:
                center_y = (left_y + right_y) / 2.0
                self.track_boundaries['centerline'].append((current_x, center_y))
            
            current_x += x_step

    def interpolate_y_at_x(self, points, target_x):
        """
        在给定X坐标处插值Y坐标
        用于中心线计算
        """
        if len(points) < 2:
            return None
        
        # 找到target_x两侧的点
        points_sorted = sorted(points, key=lambda p: p[0])
        
        for i in range(len(points_sorted) - 1):
            x1, y1 = points_sorted[i]
            x2, y2 = points_sorted[i + 1]
            
            if x1 <= target_x <= x2:
                # 线性插值
                if x2 == x1:
                    return y1
                ratio = (target_x - x1) / (x2 - x1)
                return y1 + ratio * (y2 - y1)
        
        # 如果target_x超出范围，返回最近的点
        if target_x < points_sorted[0][0]:
            return points_sorted[0][1]
        elif target_x > points_sorted[-1][0]:
            return points_sorted[-1][1]
        
        return None

    def check_mapping_completeness(self):
        """
        检查建图完整性
        评估是否有足够的信息进行路径规划
        """
        completeness_score = 0.0
        status_messages = []
        
        # 检查加速段
        if len(self.left_red_cones) >= 3:
            completeness_score += 0.25
            status_messages.append(f"左边界: {len(self.left_red_cones)}个红锥筒")
        else:
            status_messages.append(f"左边界不足: {len(self.left_red_cones)}/3个红锥筒")
        
        if len(self.right_blue_cones) >= 3:
            completeness_score += 0.25
            status_messages.append(f"右边界: {len(self.right_blue_cones)}个蓝锥筒")
        else:
            status_messages.append(f"右边界不足: {len(self.right_blue_cones)}/3个蓝锥筒")
        
        # 检查减速段
        if len(self.yellow_cones) >= 6:  # 减速段需要更多锥筒
            completeness_score += 0.25
            status_messages.append(f"减速段: {len(self.yellow_cones)}个黄锥筒")
        else:
            status_messages.append(f"减速段: {len(self.yellow_cones)}/6个黄锥筒")
        
        # 检查中心线
        if len(self.track_boundaries['centerline']) >= 10:
            completeness_score += 0.25
            status_messages.append(f"中心线: {len(self.track_boundaries['centerline'])}个点")
        else:
            status_messages.append(f"中心线不足: {len(self.track_boundaries['centerline'])}/10个点")
        
        # 发布建图状态
        status = f"建图进度: {completeness_score*100:.1f}% | " + " | ".join(status_messages)
        
        msg = String()
        msg.data = status
        self.mapping_status_pub.publish(msg)
        
        if completeness_score > 0.8:
            rospy.loginfo_throttle(5, "建图质量良好，可以进行路径规划")
        elif completeness_score > 0.5:
            rospy.loginfo_throttle(5, "建图进行中，部分路径可规划")
        else:
            rospy.logdebug("建图信息不足，等待更多锥筒检测")

    def publish_visualization(self, event):
        """
        发布建图可视化信息
        使用不同颜色区分不同类型的地图元素
        """
        # 发布分类锥筒
        self.publish_classified_cones()
        
        # 发布赛道边界
        self.publish_track_boundaries()
        
        # 发布完整地图
        self.publish_complete_map()

    def publish_classified_cones(self):
        """
        发布分类后的锥筒可视化
        红、蓝、黄锥筒分别发布到不同话题
        """
        # 红色锥筒（左边界）
        red_markers = MarkerArray()
        for i, cone in enumerate(self.left_red_cones):
            marker = self.create_cone_marker(cone, i, "red_cones", (1.0, 0.0, 0.0, 1.0))
            red_markers.markers.append(marker)
        self.red_cones_pub.publish(red_markers)
        
        # 蓝色锥筒（右边界）
        blue_markers = MarkerArray()
        for i, cone in enumerate(self.right_blue_cones):
            marker = self.create_cone_marker(cone, i, "blue_cones", (0.0, 0.0, 1.0, 1.0))
            blue_markers.markers.append(marker)
        self.blue_cones_pub.publish(blue_markers)
        
        # 黄色锥筒（减速段）
        yellow_markers = MarkerArray()
        for i, cone in enumerate(self.yellow_cones):
            marker = self.create_cone_marker(cone, i, "yellow_cones", (1.0, 1.0, 0.0, 1.0))
            yellow_markers.markers.append(marker)
        self.yellow_cones_pub.publish(yellow_markers)

    def publish_track_boundaries(self):
        """
        发布赛道边界线
        左边界（红色）、右边界（蓝色）、中心线（绿色）
        """
        marker_array = MarkerArray()
        
        # 左边界线（红色）
        if len(self.track_boundaries['left']) > 1:
            left_boundary = self.create_boundary_line(
                self.track_boundaries['left'], 0, "left_boundary", (1.0, 0.0, 0.0, 0.8)
            )
            marker_array.markers.append(left_boundary)
        
        # 右边界线（蓝色）
        if len(self.track_boundaries['right']) > 1:
            right_boundary = self.create_boundary_line(
                self.track_boundaries['right'], 1, "right_boundary", (0.0, 0.0, 1.0, 0.8)
            )
            marker_array.markers.append(right_boundary)
        
        # 中心线（绿色） - 这是建图的核心结果
        if len(self.track_boundaries['centerline']) > 1:
            centerline = self.create_boundary_line(
                self.track_boundaries['centerline'], 2, "track_centerline", (0.0, 1.0, 0.0, 1.0)
            )
            centerline.scale.x = 0.2  # 中心线稍粗一些
            marker_array.markers.append(centerline)
        
        self.boundary_pub.publish(marker_array)

    def publish_complete_map(self):
        """
        发布完整的地图信息
        包含所有建图元素的综合可视化
        """
        marker_array = MarkerArray()
        marker_id = 0
        
        # 所有锥筒的综合显示
        for cone in self.all_cones:
            marker = self.create_cone_marker(cone, marker_id, "complete_map", None)
            marker_array.markers.append(marker)
            marker_id += 1
        
        # 赛道区域标示（半透明填充）
        if (len(self.track_boundaries['left']) > 2 and 
            len(self.track_boundaries['right']) > 2):
            
            track_area = self.create_track_area_marker(marker_id, "track_area")
            if track_area:
                marker_array.markers.append(track_area)
        
        self.map_pub.publish(marker_array)

    def create_cone_marker(self, cone, marker_id, namespace, color=None):
        """
        创建锥筒标记
        统一的锥筒可视化创建函数
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # 位置
        marker.pose.position.x = cone['x']
        marker.pose.position.y = cone['y']
        marker.pose.position.z = 0.2  # 锥筒高度的一半
        marker.pose.orientation.w = 1.0
        
        # 尺寸（标准交通锥）
        marker.scale.x = 0.3  # 直径30cm
        marker.scale.y = 0.3
        marker.scale.z = 0.4  # 高度40cm
        
        # 颜色设置
        if color:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        else:
            # 根据检测到的颜色自动设置
            cone_color = cone['color'].lower()
            if cone_color in ['r', 'red']:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
            elif cone_color in ['b', 'blue']:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 1.0
            elif cone_color in ['y', 'yellow']:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.5, 0.5, 0.5, 1.0
        
        marker.lifetime = rospy.Duration(0)  # 永不过期
        
        return marker

    def create_boundary_line(self, points, marker_id, namespace, color):
        """
        创建边界线标记
        用于显示赛道边界和中心线
        """
        if len(points) < 2:
            return None
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 线条属性
        marker.scale.x = 0.1  # 线宽10cm
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        
        # 添加所有点
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.05  # 稍微抬高，避免与地面重叠
            marker.points.append(p)
        
        marker.lifetime = rospy.Duration(0)
        
        return marker

    def create_track_area_marker(self, marker_id, namespace):
        """
        创建赛道区域标记
        半透明多边形显示可行驶区域
        """
        # TODO: 实现赛道区域多边形
        # 这里需要计算左右边界围成的多边形
        return None

    def get_color_name(self, color_code):
        """
        获取颜色的中文名称
        用于日志显示
        """
        color_map = {
            'r': '红色', 'red': '红色',
            'b': '蓝色', 'blue': '蓝色',
            'y': '黄色', 'yellow': '黄色'
        }
        return color_map.get(color_code.lower(), f'未知({color_code})')

    def get_track_boundaries_for_planning(self):
        """
        为路径规划模块提供边界信息
        返回当前已建图的赛道边界
        """
        return {
            'left_boundary': self.track_boundaries['left'].copy(),
            'right_boundary': self.track_boundaries['right'].copy(),
            'centerline': self.track_boundaries['centerline'].copy(),
            'mapping_progress': self.mapping_progress,
            'sections_detected': self.track_sections.copy()
        }

if __name__ == '__main__':
    try:
        mapper = DynamicMapper()
        rospy.loginfo("动态建图模块运行中，开始实时建图...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("动态建图模块正常退出")
    except Exception as e:
        rospy.logerr(f"动态建图模块错误: {e}")
        import traceback
        traceback.print_exc()