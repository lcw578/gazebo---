#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能坐标系转换器
基于分析结果自动转换建图坐标系到Gazebo坐标系
"""
# filepath: e:\gazebo大作业\catkin_ws\catkin_ws\src\fsac_autonomous\scripts\mapping\smart_coordinate_transformer.py

import rospy
import tf2_ros
import numpy as np
import math
import json
import pickle
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

class SmartCoordinateTransformer:
    def __init__(self):
        rospy.init_node('smart_coordinate_transformer', anonymous=True)
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 发布器
        self.status_pub = rospy.Publisher('/transform/smart_status', String, queue_size=10)
        self.gazebo_path_pub = rospy.Publisher('/transform/gazebo_aligned_path', Path, queue_size=10)
        self.transform_ready_pub = rospy.Publisher('/transform/ready', Bool, queue_size=10)
        
        # 数据
        self.original_data = []
        self.transformed_data = []
        self.transform_params = {}
        self.ready = False
        
        # Gazebo参数
        self.gazebo_start_x = rospy.get_param('/diff_drive/initial_x', -0.55)
        self.gazebo_start_y = rospy.get_param('/diff_drive/initial_y', 0.0)
        self.gazebo_start_yaw = math.radians(rospy.get_param('/diff_drive/initial_yaw_degrees', 0.0))
        
        rospy.loginfo("🤖 智能坐标转换器启动")
        rospy.loginfo(f"🎯 Gazebo目标位置: ({self.gazebo_start_x:.3f}, {self.gazebo_start_y:.3f}), 朝向: {math.degrees(self.gazebo_start_yaw):.1f}°")
        
        # 等待分析结果，然后执行转换
        rospy.sleep(1.0)  # 等待分析器启动
        if self.load_and_transform():
            self.ready = True
            rospy.loginfo("✅ 智能坐标转换完成")
        
        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)

    def load_and_transform(self):
        """载入数据并执行智能转换"""
        # 载入原始数据
        if not self.load_original_data():
            return False
        
        # 载入分析结果
        analysis_result = self.load_analysis_result()
        
        # 执行转换
        return self.perform_smart_transform(analysis_result)

    def load_original_data(self):
        """载入原始建图数据"""
        map_files = [
            '/tmp/track_map.pkl',
            '/tmp/track_map.json',
            '/tmp/centerline.pkl',
            '/tmp/centerline.json'
        ]
        
        for map_file in map_files:
            try:
                if map_file.endswith('.pkl'):
                    with open(map_file, 'rb') as f:
                        data = pickle.load(f)
                else:
                    with open(map_file, 'r') as f:
                        data = json.load(f)
                
                if 'centerline' in data:
                    self.original_data = data['centerline']
                elif isinstance(data, list):
                    self.original_data = data
                else:
                    continue
                    
                if len(self.original_data) > 5:
                    rospy.loginfo(f"✅ 载入原始数据: {len(self.original_data)} 点")
                    return True
                    
            except Exception as e:
                rospy.logdebug(f"载入失败 {map_file}: {e}")
        
        rospy.logerr("❌ 未找到原始建图数据")
        return False

    def load_analysis_result(self):
        """载入坐标系分析结果"""
        # 这里可以从分析器获取结果，或者自己重新分析
        return self.quick_analysis()

    def quick_analysis(self):
        """快速分析建图数据特征"""
        if not self.original_data:
            return None
        
        # 提取坐标
        x_coords = []
        y_coords = []
        
        for point in self.original_data:
            if isinstance(point, dict):
                x_coords.append(point.get('x', 0))
                y_coords.append(point.get('y', 0))
            else:
                x_coords.append(point[0])
                y_coords.append(point[1])
        
        # 计算运动方向
        directions = []
        for i in range(1, min(len(x_coords), 20)):  # 只分析前20个点
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                directions.append(math.atan2(dy, dx))
        
        if len(directions) > 0:
            mean_direction = np.mean(directions)
            mean_direction_deg = math.degrees(mean_direction)
        else:
            mean_direction = 0.0
            mean_direction_deg = 0.0
        
        # 推测需要的旋转角度
        rotation_needed = self.estimate_rotation_needed(mean_direction_deg)
        
        return {
            'start_point': [x_coords[0], y_coords[0]],
            'mean_direction_deg': mean_direction_deg,
            'rotation_needed_deg': rotation_needed,
            'data_range': {
                'x_span': max(x_coords) - min(x_coords),
                'y_span': max(y_coords) - min(y_coords)
            }
        }

    def estimate_rotation_needed(self, mean_direction_deg):
        """估计需要的旋转角度"""
        # 目标：让建图轨迹的主方向对齐到Gazebo的期望方向
        target_direction_deg = math.degrees(self.gazebo_start_yaw)  # Gazebo中期望的朝向
        
        # 计算需要的旋转角度
        rotation_needed = target_direction_deg - mean_direction_deg
        
        # 规范化到[-180, 180]
        while rotation_needed > 180:
            rotation_needed -= 360
        while rotation_needed < -180:
            rotation_needed += 360
        
        rospy.loginfo(f"🧭 方向分析:")
        rospy.loginfo(f"   建图主方向: {mean_direction_deg:.1f}°")
        rospy.loginfo(f"   目标方向: {target_direction_deg:.1f}°")
        rospy.loginfo(f"   需要旋转: {rotation_needed:.1f}°")
        
        return rotation_needed

    def perform_smart_transform(self, analysis):
        """执行智能坐标转换"""
        if not analysis or not self.original_data:
            return False
        
        rospy.loginfo("🔄 执行智能坐标转换...")
        
        # 提取转换参数
        start_point = analysis['start_point']
        rotation_deg = analysis['rotation_needed_deg']
        rotation_rad = math.radians(rotation_deg)
        
        # 计算平移量（让建图起点对齐到Gazebo起点）
        translation_x = self.gazebo_start_x - start_point[0]
        translation_y = self.gazebo_start_y - start_point[1]
        
        # 保存转换参数
        self.transform_params = {
            'translation': [translation_x, translation_y, 0.0],
            'rotation_rad': rotation_rad,
            'rotation_deg': rotation_deg,
            'gazebo_target': [self.gazebo_start_x, self.gazebo_start_y, self.gazebo_start_yaw],
            'map_origin': start_point
        }
        
        # 应用转换
        self.apply_transformation()
        
        # 发布TF变换
        self.publish_tf_transform()
        
        # 保存结果
        self.save_transformed_data()
        
        rospy.loginfo(f"✅ 转换完成:")
        rospy.loginfo(f"   平移: ({translation_x:.3f}, {translation_y:.3f})")
        rospy.loginfo(f"   旋转: {rotation_deg:.1f}°")
        
        return True

    def apply_transformation(self):
        """应用坐标转换"""
        self.transformed_data = []
        
        tx, ty, tz = self.transform_params['translation']
        rotation = self.transform_params['rotation_rad']
        
        cos_r = math.cos(rotation)
        sin_r = math.sin(rotation)
        
        for point in self.original_data:
            # 提取原始坐标
            if isinstance(point, dict):
                x_orig = point.get('x', 0)
                y_orig = point.get('y', 0)
                extra = {k: v for k, v in point.items() if k not in ['x', 'y']}
            else:
                x_orig = point[0]
                y_orig = point[1]
                extra = {}
            
            # 先旋转，再平移
            x_rot = x_orig * cos_r - y_orig * sin_r
            y_rot = x_orig * sin_r + y_orig * cos_r
            
            x_final = x_rot + tx
            y_final = y_rot + ty
            
            # 计算变换后的朝向
            if len(self.transformed_data) > 0:
                prev = self.transformed_data[-1]
                dx = x_final - prev['x']
                dy = y_final - prev['y']
                if abs(dx) > 0.01 or abs(dy) > 0.01:
                    theta = math.atan2(dy, dx)
                else:
                    theta = extra.get('theta', 0.0)
            else:
                theta = extra.get('theta', 0.0)
            
            transformed_point = {
                'x': x_final,
                'y': y_final,
                'theta': theta,
                **extra
            }
            
            self.transformed_data.append(transformed_point)

    def publish_tf_transform(self):
        """发布TF变换"""
        transform = TransformStamped()
        
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "odom"  # Gazebo坐标系
        transform.child_frame_id = "map"    # 建图坐标系
        
        # 设置变换（注意：这是从map到odom的变换，所以要取逆）
        tx, ty, tz = self.transform_params['translation']
        rotation = self.transform_params['rotation_rad']
        
        # 逆变换
        transform.transform.translation.x = -tx * math.cos(-rotation) + ty * math.sin(-rotation)
        transform.transform.translation.y = -tx * math.sin(-rotation) - ty * math.cos(-rotation)
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(-rotation / 2.0)
        transform.transform.rotation.w = math.cos(-rotation / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self, event):
        """发布状态信息"""
        if not self.ready:
            status = "⏳ 智能坐标转换进行中..."
        else:
            params = self.transform_params
            status = f"✅ 智能转换完成: " \
                    f"平移({params['translation'][0]:.2f}, {params['translation'][1]:.2f}), " \
                    f"旋转{params['rotation_deg']:.1f}°, " \
                    f"共{len(self.transformed_data)}点"
        
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
        # 发布准备状态
        ready_msg = Bool()
        ready_msg.data = self.ready
        self.transform_ready_pub.publish(ready_msg)
        
        # 发布转换后的路径
        if self.ready:
            self.publish_gazebo_path()

    def publish_gazebo_path(self):
        """发布转换后的Gazebo路径"""
        if not self.transformed_data:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = "odom"  # Gazebo坐标系
        path_msg.header.stamp = rospy.Time.now()
        
        for point in self.transformed_data:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            
            # 设置朝向
            theta = point.get('theta', 0.0)
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            
            path_msg.poses.append(pose)
        
        self.gazebo_path_pub.publish(path_msg)

    def save_transformed_data(self):
        """保存转换后的数据"""
        try:
            output_data = {
                'transformed_centerline': self.transformed_data,
                'transform_parameters': self.transform_params,
                'metadata': {
                    'timestamp': rospy.Time.now().to_sec(),
                    'source': 'smart_coordinate_transformer',
                    'point_count': len(self.transformed_data),
                    'coordinate_system': 'gazebo_odom'
                }
            }
            
            # 保存JSON格式
            with open('/tmp/gazebo_aligned_centerline.json', 'w') as f:
                json.dump(output_data, f, indent=2)
            
            # 保存pickle格式
            with open('/tmp/gazebo_aligned_centerline.pkl', 'wb') as f:
                pickle.dump(output_data, f)
            
            rospy.loginfo("💾 转换数据已保存到 /tmp/gazebo_aligned_centerline.*")
            
        except Exception as e:
            rospy.logerr(f"❌ 保存失败: {e}")

    def get_transformed_centerline(self):
        """获取转换后的中心线（供其他模块调用）"""
        return self.transformed_data

if __name__ == '__main__':
    try:
        transformer = SmartCoordinateTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("智能坐标转换器关闭")