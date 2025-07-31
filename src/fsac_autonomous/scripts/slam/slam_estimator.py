#!/usr/bin/env python3
# filepath: src/fsac_autonomous/scripts/slam/slam_estimator.py
# -*- coding: utf-8 -*-
"""
SLAM状态估计器
- 融合里程计和锥筒观测
- 估计车辆状态和锥筒位置
- 发布状态信息
"""
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from fsd_common_msgs.msg import CarState, ConeDetections
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point

class SLAMEstimator:
    def __init__(self):
        rospy.init_node('slam_estimator', anonymous=True)
        
        # 参数
        self.use_odom_prediction = rospy.get_param('~use_odom_prediction', True)
        self.landmark_association_threshold = rospy.get_param('~landmark_association_threshold', 1.0)
        self.state_update_rate = rospy.get_param('~state_update_rate', 50)
        
        # 状态变量
        self.car_state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vtheta': 0.0
        }
        
        self.last_odom = None
        self.landmarks = {}  # 地标地图
        
        # 订阅器
        if self.use_odom_prediction:
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        
        # 修正：订阅正确的锥筒话题
        self.cones_sub = rospy.Subscriber('/perception/lidar/cone_detections', ConeDetections, 
                                        self.cones_callback, queue_size=10)
        
        # 发布器
        self.state_pub = rospy.Publisher('/estimation/slam/state', CarState, queue_size=10)
        
        # 定时器
        self.state_timer = rospy.Timer(rospy.Duration(1.0/self.state_update_rate), 
                                     self.publish_state)
        
        rospy.loginfo("SLAM状态估计器启动完成")

    def odom_callback(self, odom_msg):
        """里程计回调"""
        if self.last_odom is None:
            self.last_odom = odom_msg
            return
        
        # 简单的里程计积分
        dt = (odom_msg.header.stamp - self.last_odom.header.stamp).to_sec()
        if dt > 0:
            # 更新位置
            self.car_state['x'] = odom_msg.pose.pose.position.x
            self.car_state['y'] = odom_msg.pose.pose.position.y
            
            # 更新姿态（从四元数转换）
            orientation = odom_msg.pose.pose.orientation
            self.car_state['theta'] = self.quaternion_to_yaw(orientation)
            
            # 更新速度
            self.car_state['vx'] = odom_msg.twist.twist.linear.x
            self.car_state['vy'] = odom_msg.twist.twist.linear.y
            self.car_state['vtheta'] = odom_msg.twist.twist.angular.z
        
        self.last_odom = odom_msg

    def cones_callback(self, cones_msg):
        """锥筒检测回调"""
        try:
            # 修改这里：使用正确的字段名
            global_cones = []
            for cone in cones_msg.cone_detections:
                # 将锥筒转换到地图坐标系
                map_point = self.transform_to_map(cone.position)
                if map_point:
                    # 根据颜色分类添加地标
                    color = cone.color.data if hasattr(cone.color, 'data') else str(cone.color)
                    global_cone = {
                        'x': map_point.x,
                        'y': map_point.y,
                        'color': color
                    }
                    global_cones.append(global_cone)
            
            # 更新地标地图
            self.update_landmarks(global_cones)
            
        except Exception as e:
            rospy.logwarn(f"SLAM估计出错: {e}")

    def transform_to_map(self, cone_position):
        """将锥筒位置转换到地图坐标系"""
        try:
            # 当前车辆状态
            car_x = self.car_state['x']
            car_y = self.car_state['y']
            car_theta = self.car_state['theta']
            
            # 锥筒在车辆坐标系中的位置
            cone_x_local = cone_position.x
            cone_y_local = cone_position.y
            
            # 转换到地图坐标系
            cos_theta = math.cos(car_theta)
            sin_theta = math.sin(car_theta)
            
            map_x = car_x + cone_x_local * cos_theta - cone_y_local * sin_theta
            map_y = car_y + cone_x_local * sin_theta + cone_y_local * cos_theta
            
            map_point = Point()
            map_point.x = map_x
            map_point.y = map_y
            map_point.z = 0.0
            
            return map_point
            
        except Exception as e:
            rospy.logwarn(f"坐标转换失败: {e}")
            return None

    def add_landmark(self, map_point, color):
        """添加地标"""
        landmark = {
            'x': map_point.x,
            'y': map_point.y,
            'color': color
        }
        self.update_landmarks([landmark])

    def update_map(self):
        """更新地图（占位函数）"""
        pass

    def quaternion_to_yaw(self, orientation):
        """四元数转换为偏航角"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update_landmarks(self, observed_cones):
        """更新地标地图"""
        for obs_cone in observed_cones:
            # 查找匹配的已知地标
            matched_id = self.find_matching_landmark(obs_cone)
            
            if matched_id is not None:
                # 更新已知地标
                self.landmarks[matched_id]['x'] = obs_cone['x']
                self.landmarks[matched_id]['y'] = obs_cone['y']
                self.landmarks[matched_id]['observations'] += 1
            else:
                # 添加新地标
                new_id = len(self.landmarks)
                self.landmarks[new_id] = {
                    'x': obs_cone['x'],
                    'y': obs_cone['y'],
                    'color': obs_cone['color'],
                    'observations': 1
                }

    def find_matching_landmark(self, observed_cone):
        """查找匹配的地标"""
        min_distance = float('inf')
        matched_id = None
        
        for landmark_id, landmark in self.landmarks.items():
            distance = math.sqrt(
                (landmark['x'] - observed_cone['x'])**2 + 
                (landmark['y'] - observed_cone['y'])**2
            )
            
            if (distance < self.landmark_association_threshold and 
                distance < min_distance and
                landmark['color'] == observed_cone['color']):
                min_distance = distance
                matched_id = landmark_id
        
        return matched_id

    def publish_state(self, event):
        """发布车辆状态"""
        state_msg = CarState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = "map"
        
        # 车辆位置和姿态
        state_msg.car_state.x = self.car_state['x']
        state_msg.car_state.y = self.car_state['y']
        state_msg.car_state.theta = self.car_state['theta']
        
        # 车辆速度
        state_msg.car_state_dt.car_state_dt.x = self.car_state['vx']
        state_msg.car_state_dt.car_state_dt.y = self.car_state['vy']
        state_msg.car_state_dt.car_state_dt.theta = self.car_state['vtheta']
        
        self.state_pub.publish(state_msg)
        
        # 记录状态信息
        speed = math.sqrt(self.car_state['vx']**2 + self.car_state['vy']**2)
        rospy.loginfo_throttle(5.0, 
            f"车辆状态: 位置({self.car_state['x']:.2f}, {self.car_state['y']:.2f}), "
            f"速度{speed:.2f}m/s, 地标{len(self.landmarks)}个")

if __name__ == '__main__':
    try:
        estimator = SLAMEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("SLAM状态估计器正常退出")