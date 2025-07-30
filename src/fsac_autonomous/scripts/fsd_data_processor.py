#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np

# 标准ROS消息
from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import Point, Pose2D, PoseStamped, TransformStamped, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage

# 自定义消息类型（用于接收bag数据）
from fsd_common_msgs.msg import CarState, ConeDetections, Cone, CarStateDt

class FSDDataProcessor:
    def __init__(self):
        """初始化FSD数据处理节点"""
        rospy.init_node('fsd_data_processor', anonymous=True)
        
        # =================================================================
        # TF系统设置
        # =================================================================
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # =================================================================
        # 订阅器设置 - 接收bag文件数据
        # =================================================================
        self.car_state_sub = rospy.Subscriber(
            '/estimation/slam/state',
            CarState,
            self.car_state_callback,
            queue_size=50
        )
        
        self.cone_detections_sub = rospy.Subscriber(
            '/perception/lidar/cone_detections',
            ConeDetections,
            self.cone_detections_callback,
            queue_size=50
        )
        
        # =================================================================
        # 发布器设置 - 输出标准ROS消息
        # =================================================================
        # 车辆状态相关
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('/fsac/vehicle_pose', PoseStamped, queue_size=10)
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=100)
        
        # 锥筒和地图相关
        self.cones_pub = rospy.Publisher('/fsac/cone_markers', MarkerArray, queue_size=10)
        self.left_boundary_pub = rospy.Publisher('/fsac/left_boundary', MarkerArray, queue_size=10)
        self.right_boundary_pub = rospy.Publisher('/fsac/right_boundary', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/fsac/planned_path', Path, queue_size=10)
        
        # 调试和统计信息
        self.debug_pub = rospy.Publisher('/fsac/debug_info', String, queue_size=10)
        
        # =================================================================
        # 数据存储
        # =================================================================
        self.detected_cones = []  # 全局锥筒列表
        self.current_car_state = None  # 当前车辆状态
        self.left_boundary_cones = []  # 左边界锥筒
        self.right_boundary_cones = []  # 右边界锥筒
        
        # 统计信息
        self.total_messages_received = 0
        self.total_cones_detected = 0
        
        # =================================================================
        # 配置参数
        # =================================================================
        self.map_frame = "map"
        self.base_frame = "base_link"
        self.laser_frame = "laser_link"
        self.odom_frame = "odom"
        
        # 传感器安装位置（题目中提到lidar到惯导距离2.4m）
        self.lidar_offset_x = 2.4
        self.lidar_offset_y = 0.0
        self.lidar_offset_z = 0.0
        
        # 锥筒过滤参数
        self.min_cone_confidence = 0.3  # 最小置信度阈值
        self.cone_duplicate_distance = 0.8  # 重复检测距离阈值
        
        # =================================================================
        # 定时器设置
        # =================================================================
        # 定期发布静态TF变换
        self.tf_timer = rospy.Timer(rospy.Duration(0.1), self.publish_static_transforms)
        
        # 定期更新路径规划
        self.path_timer = rospy.Timer(rospy.Duration(0.5), self.update_path_planning)
        
        # 定期发布统计信息
        self.stats_timer = rospy.Timer(rospy.Duration(2.0), self.publish_statistics)
        
        rospy.loginfo("=== FSD数据处理节点启动完成 ===")
        rospy.loginfo(f"监听话题: /estimation/slam/state, /perception/lidar/cone_detections")
        rospy.loginfo(f"发布坐标系: {self.map_frame} -> {self.base_frame}")
        rospy.loginfo(f"激光雷达偏移: ({self.lidar_offset_x}, {self.lidar_offset_y}, {self.lidar_offset_z})")

    def car_state_callback(self, msg):
        """
        处理车辆状态数据回调函数
        msg: fsac_autonomous/CarState
        """
        self.current_car_state = msg
        self.total_messages_received += 1
        
        # 发布TF变换
        self.publish_vehicle_tf(msg)
        
        # 发布里程计信息
        self.publish_odometry(msg)
        
        # 发布车辆位姿
        self.publish_vehicle_pose(msg)
        
        # 调试信息
        if self.total_messages_received % 100 == 1:  # 每100个消息打印一次
            rospy.loginfo(f"车辆位置: ({msg.car_state.x:.2f}, {msg.car_state.y:.2f}, {msg.car_state.theta:.2f})")
            rospy.loginfo(f"车辆速度: ({msg.car_state_dt.car_state_dt.x:.2f}, {msg.car_state_dt.car_state_dt.y:.2f})")

    def cone_detections_callback(self, msg):
        """
        处理锥筒检测数据回调函数
        msg: fsac_autonomous/ConeDetections
        """
        if self.current_car_state is None:
            rospy.logwarn("还未收到车辆状态数据，跳过锥筒处理")
            return
        
        rospy.loginfo(f"收到 {len(msg.cone_detections)} 个锥筒检测")
        
        # 获取当前车辆状态
        car_x = self.current_car_state.car_state.x
        car_y = self.current_car_state.car_state.y
        car_theta = self.current_car_state.car_state.theta
        
        # 处理每个检测到的锥筒
        new_cones_count = 0
        
        for idx, cone in enumerate(msg.cone_detections):
            # 打印原始检测数据
            rospy.logdebug(f"  锥筒 {idx}: "
                          f"激光雷达坐标=({cone.position.x:.2f}, {cone.position.y:.2f}), "
                          f"颜色='{cone.color.data}', "
                          f"位置置信度={cone.poseConfidence.data:.2f}, "
                          f"颜色置信度={cone.colorConfidence.data:.2f}")
            
            # 检查置信度
            if cone.poseConfidence.data < self.min_cone_confidence:
                rospy.logdebug(f"    ✗ 置信度过低: {cone.poseConfidence.data:.2f} < {self.min_cone_confidence}")
                continue
            
            # 坐标变换：激光雷达坐标系 -> 车辆坐标系 -> 世界坐标系
            cone_world = self.transform_cone_to_world(cone, car_x, car_y, car_theta)
            
            # 检查是否是重复锥筒
            if self.is_duplicate_cone(cone_world):
                rospy.logdebug(f"    ✗ 重复锥筒，跳过")
                continue
            
            # 添加新锥筒
            self.detected_cones.append(cone_world)
            new_cones_count += 1
            self.total_cones_detected += 1
            
            rospy.loginfo(f"    ✓ 新{self.get_color_name(cone_world['color'])}锥筒: "
                         f"世界坐标({cone_world['x']:.2f}, {cone_world['y']:.2f})")
        
        if new_cones_count > 0:
            rospy.loginfo(f"本次新增 {new_cones_count} 个锥筒，总计 {len(self.detected_cones)} 个")
            
            # 分离左右边界
            self.separate_cone_boundaries()
            
            # 发布锥筒可视化
            self.publish_cone_visualizations()

    def transform_cone_to_world(self, cone, car_x, car_y, car_theta):
        """
        将锥筒从激光雷达坐标系转换到世界坐标系
        """
        # 激光雷达坐标系中的锥筒位置
        cone_x_lidar = cone.position.x
        cone_y_lidar = cone.position.y
        
        # 转换到车辆坐标系（考虑激光雷达安装偏移）
        cone_x_vehicle = cone_x_lidar + self.lidar_offset_x
        cone_y_vehicle = cone_y_lidar + self.lidar_offset_y
        
        # 转换到世界坐标系
        cos_theta = math.cos(car_theta)
        sin_theta = math.sin(car_theta)
        
        cone_x_world = car_x + cone_x_vehicle * cos_theta - cone_y_vehicle * sin_theta
        cone_y_world = car_y + cone_x_vehicle * sin_theta + cone_y_vehicle * cos_theta
        
        # 创建锥筒数据结构
        cone_data = {
            'x': cone_x_world,
            'y': cone_y_world,
            'z': 0.0,
            'color': cone.color.data,
            'pose_confidence': cone.poseConfidence.data,
            'color_confidence': cone.colorConfidence.data,
            'timestamp': rospy.Time.now(),
            'lidar_x': cone_x_lidar,
            'lidar_y': cone_y_lidar
        }
        
        return cone_data

    def is_duplicate_cone(self, new_cone):
        """检查是否是重复的锥筒"""
        for existing_cone in self.detected_cones:
            distance = math.sqrt(
                (new_cone['x'] - existing_cone['x'])**2 + 
                (new_cone['y'] - existing_cone['y'])**2
            )
            if distance < self.cone_duplicate_distance:
                return True
        return False

    def separate_cone_boundaries(self):
        """
        分离左右边界锥筒
        简化算法：基于颜色和y坐标进行分离
        """
        blue_cones = []
        yellow_cones = []
        red_cones = []
        
        for cone in self.detected_cones:
            if cone['color'] == 'b':
                blue_cones.append(cone)
            elif cone['color'] == 'y':
                yellow_cones.append(cone)
            elif cone['color'] == 'r':
                red_cones.append(cone)
        
        
        self.left_boundary_cones = sorted(red_cones, key=lambda c: c['x'])
        self.right_boundary_cones = sorted(blue_cones, key=lambda c: c['x'])
        
        rospy.logdebug(f"边界分离: 左边界(红色)={len(self.left_boundary_cones)}, "
                      f"右边界(蓝色)={len(self.right_boundary_cones)}, "
                      f"黄色锥筒={len(red_cones)}")

    def publish_vehicle_tf(self, car_state):
        """发布车辆TF变换"""
        # 创建 map -> base_link 变换
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        
        # 位置设置
        t.transform.translation.x = car_state.car_state.x
        t.transform.translation.y = car_state.car_state.y
        t.transform.translation.z = 0.0
        
        # 姿态设置（从欧拉角转换为四元数）
        theta = car_state.car_state.theta
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        
        # 广播TF
        self.tf_broadcaster.sendTransform(t)
        
        # 同时发布到/tf话题
        tf_msg = TFMessage()
        tf_msg.transforms.append(t)
        self.tf_pub.publish(tf_msg)

    def publish_static_transforms(self, event):
        """发布静态TF变换"""
        transforms = []
        
        # base_link -> laser_link 变换
        t_laser = TransformStamped()
        t_laser.header.stamp = rospy.Time.now()
        t_laser.header.frame_id = self.base_frame
        t_laser.child_frame_id = self.laser_frame
        t_laser.transform.translation.x = self.lidar_offset_x
        t_laser.transform.translation.y = self.lidar_offset_y
        t_laser.transform.translation.z = self.lidar_offset_z
        t_laser.transform.rotation.w = 1.0
        transforms.append(t_laser)
        
        # odom -> map 变换（简化为单位变换）
        t_odom = TransformStamped()
        t_odom.header.stamp = rospy.Time.now()
        t_odom.header.frame_id = self.map_frame
        t_odom.child_frame_id = self.odom_frame
        t_odom.transform.rotation.w = 1.0
        transforms.append(t_odom)
        
        # 广播所有变换
        for t in transforms:
            self.tf_broadcaster.sendTransform(t)

    def publish_odometry(self, car_state):
        """发布里程计信息"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # 位置信息
        odom.pose.pose.position.x = car_state.car_state.x
        odom.pose.pose.position.y = car_state.car_state.y
        odom.pose.pose.position.z = 0.0
        
        # 姿态信息
        theta = car_state.car_state.theta
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # 速度信息
        odom.twist.twist.linear.x = car_state.car_state_dt.car_state_dt.x
        odom.twist.twist.linear.y = car_state.car_state_dt.car_state_dt.y
        odom.twist.twist.angular.z = car_state.car_state_dt.car_state_dt.theta
        
        self.odom_pub.publish(odom)

    def publish_vehicle_pose(self, car_state):
        """发布车辆位姿"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        
        pose.pose.position.x = car_state.car_state.x
        pose.pose.position.y = car_state.car_state.y
        pose.pose.position.z = 0.0
        
        theta = car_state.car_state.theta
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.pose_pub.publish(pose)

    def publish_cone_visualizations(self):
        """发布锥筒可视化标记"""
        # 发布所有锥筒
        self.publish_all_cones_markers()
        
        # 发布左右边界
        self.publish_boundary_markers()

    def publish_all_cones_markers(self):
        """发布所有锥筒的可视化标记"""
        marker_array = MarkerArray()
        
        # 删除旧标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 添加每个锥筒的标记
        for i, cone in enumerate(self.detected_cones):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"cones_{cone['color']}"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 位置设置
            marker.pose.position.x = cone['x']
            marker.pose.position.y = cone['y']
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            
            # 尺寸设置
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.4
            
            # 颜色设置
            if cone['color'] == 'b':  # 蓝色
                marker.color.r = 0.0
                marker.color.g = 0.3
                marker.color.b = 1.0
            elif cone['color'] == 'y':  # 黄色
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif cone['color'] == 'r':  # 红色
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
 
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            
            marker_array.markers.append(marker)
        
        self.cones_pub.publish(marker_array)

    def publish_boundary_markers(self):
        """发布边界标记 - 适配你的赛道布局"""
        # 发布左边界（红色锥筒）- 用深红色球体标记
        left_markers = MarkerArray()
        for i, cone in enumerate(self.left_boundary_cones):
            marker = self.create_boundary_marker(cone, i, "left_boundary", (0.8, 0.0, 0.0))  # 深红色
            left_markers.markers.append(marker)
        self.left_boundary_pub.publish(left_markers)
        
        # 发布右边界（蓝色锥筒）- 用深蓝色球体标记
        right_markers = MarkerArray()
        for i, cone in enumerate(self.right_boundary_cones):
            marker = self.create_boundary_marker(cone, i, "right_boundary", (0.0, 0.0, 0.8))  # 深蓝色
            right_markers.markers.append(marker)
        self.right_boundary_pub.publish(right_markers)

    def create_boundary_marker(self, cone, marker_id, namespace, color):
        """创建边界标记"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = cone['x']
        marker.pose.position.y = cone['y']
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(0)
        return marker

    def update_path_planning(self, event):
        """更新路径规划"""
        if len(self.left_boundary_cones) < 2 or len(self.right_boundary_cones) < 2:
            return
        
        # 生成中心线路径
        path = self.generate_centerline_path()
        if path:
            self.path_pub.publish(path)

    def generate_centerline_path(self):
        """生成中心线路径"""
        if len(self.left_boundary_cones) < 2 or len(self.right_boundary_cones) < 2:
            return None
        
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        
        # 简单算法：取左右边界对应点的中点
        min_len = min(len(self.left_boundary_cones), len(self.right_boundary_cones))
        
        for i in range(min_len):
            left_cone = self.left_boundary_cones[i]
            right_cone = self.right_boundary_cones[i]
            
            # 计算中点
            center_x = (left_cone['x'] + right_cone['x']) / 2.0
            center_y = (left_cone['y'] + right_cone['y']) / 2.0
            
            # 创建路径点
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = center_x
            pose.pose.position.y = center_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        return path

    def publish_statistics(self, event):
        """发布统计信息"""
        stats = f"统计信息 - 消息总数: {self.total_messages_received}, " \
                f"锥筒总数: {len(self.detected_cones)}, " \
                f"左边界: {len(self.left_boundary_cones)}, " \
                f"右边界: {len(self.right_boundary_cones)}"
        
        msg = String()
        msg.data = stats
        self.debug_pub.publish(msg)
        
        rospy.loginfo(stats)

    def get_color_name(self, color_code):
        """获取颜色名称"""
        color_map = {'b': '蓝色', 'y': '黄色', 'o': '橙色'}
        return color_map.get(color_code, '未知')

if __name__ == '__main__':
    try:
        processor = FSDDataProcessor()
        rospy.loginfo("FSD数据处理器运行中...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("FSD数据处理器正常退出")
    except Exception as e:
        rospy.logerr(f"FSD数据处理器异常退出: {e}")