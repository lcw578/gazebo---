#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能路径规划模块 - 75米直线加速赛 🏁 (已修复版)
功能：
1. 自动读取初始偏角并生成强制收敛的纠偏曲线
2. 0-75米三段式加速冲刺
3. 75米后紧急制动系统
4. 实时动态路径重规划（优化触发条件）
5. 基于偏离程度的自适应速度控制
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from fsd_common_msgs.msg import CarState

class SimplePathPlanner:
    def __init__(self):
        rospy.init_node('simple_path_planner', anonymous=True)
        
        # 🏁 赛道参数 - 针对3米窄赛道优化
        self.track_width = 3.0            # 3米窄赛道
        self.acceleration_length = 75.0   # 加速段：75米
        self.track_length = 175.0         # 总长度：175米
        self.correction_distance = 8.0    # 纠偏距离：从15米缩短到8米 ⚡
        
        # 🚀 速度参数 - 更保守的速度设置
        self.target_speed_correction = 2.5   # 纠偏速度：降低到2.5m/s
        self.target_speed_normal = 5.0       # 正常速度：降低到5.0m/s
        self.target_speed_max = 8.0          # 最大速度：降低到8.0m/s
        
        # 📍 当前状态
        self.current_car_state = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'v_x': 0.0,
            'v_y': 0.0
        }
        self.current_target_speed = 0.0
        self.correction_complete = False
        self.emergency_stop_triggered = False
        
        # 🛣️ 路径存储
        self.planned_path = []
        self.initial_yaw_degrees = 0.0
        
        # 📡 订阅器
        self.state_sub = rospy.Subscriber('/estimation/slam/state', CarState, 
                                        self.car_state_callback, queue_size=10)
        
        # 📢 发布器
        self.path_pub = rospy.Publisher('/planning/global_path', Path, queue_size=10)
        self.planned_path_pub = rospy.Publisher('/planning/planned_path', Path, queue_size=10)
        self.planning_line_pub = rospy.Publisher('/planning/planning_line', Path, queue_size=10)
        self.target_speed_pub = rospy.Publisher('/planning/target_speed', Float32, queue_size=10)
        self.status_pub = rospy.Publisher('/planning/status', String, queue_size=10)
        
        # 🎨 可视化发布器
        self.markers_pub = rospy.Publisher('/planning/debug_markers', MarkerArray, queue_size=10)
        self.target_point_pub = rospy.Publisher('/planning/target_point', PoseStamped, queue_size=10)
        
        # 🔄 延迟初始化路径（等待参数）
        rospy.Timer(rospy.Duration(1.0), self.delayed_initialization, oneshot=True)
        
        # ⏱️ 主循环定时器 - 提高频率到20Hz
        self.planning_timer = rospy.Timer(rospy.Duration(0.05), self.planning_loop)
        
        # 🚨 紧急停车定时器
        self.emergency_timer = rospy.Timer(rospy.Duration(0.02), self.emergency_safety_check)
        
        rospy.loginfo("🏁 智能路径规划器启动完成！")

    def delayed_initialization(self, event):
        """🔄 延迟初始化 - 等待参数服务器"""
        self.read_initial_parameters()
        self.generate_optimal_path()

    def read_initial_parameters(self):
        """📖 读取初始参数 - 移除测试代码"""
        try:
            # 等待参数设置
            rospy.sleep(2.0)
            
            # 读取初始偏角（由generate_yaw.py设置）
            if rospy.has_param('/fsac/initial_yaw_degrees'):
                self.initial_yaw_degrees = rospy.get_param('/fsac/initial_yaw_degrees', 0.0)
                rospy.loginfo(f"🎯 读取到初始偏角: {self.initial_yaw_degrees:.2f}°")
            else:
                rospy.logwarn("⚠️ 未找到初始偏角参数，使用默认值0°")
                self.initial_yaw_degrees = 0.0
                
        except Exception as e:
            rospy.logwarn(f"读取参数失败，使用默认值: {e}")
            self.initial_yaw_degrees = 0.0

    def generate_optimal_path(self):
        """🛣️ 生成最优路径 - 基于当前实际状态"""
        # ✅ 使用当前车辆的实际状态，而不是启动时的参数
        car_x = self.current_car_state['x']
        car_y = self.current_car_state['y']
        car_theta = self.current_car_state['theta']  # 当前实际角度
        
        rospy.loginfo(f"🔧 基于当前状态生成路径: 位置({car_x:.2f}, {car_y:.2f}), 角度{math.degrees(car_theta):.2f}°")
        
        # 清空路径
        self.planned_path = []
        
        # 📐 动态计算纠偏需求
        lateral_error = abs(car_y)
        angle_error = abs(math.degrees(car_theta))
        
        # 🎯 从当前位置开始规划
        start_x = max(0.0, car_x)
        
        # 🔧 动态纠偏距离
        if lateral_error > 0.1 or angle_error > 0.5:
            # 根据偏离程度确定纠偏距离
            correction_distance = min(12.0, 5.0 + lateral_error * 3.0 + angle_error * 0.2)
            correction_end_x = start_x + correction_distance
            
            # ✅ 使用当前实际状态生成纠偏曲线
            correction_curve = self.generate_correction_curve(
                start_x, correction_end_x, car_theta, car_y  # 传入当前状态
            )
            self.planned_path.extend(correction_curve)
            rospy.loginfo(f"✅ 生成纠偏曲线: {start_x:.1f}→{correction_end_x:.1f}m (距离{correction_distance:.1f}m)")
        else:
            # 无需纠偏，直接生成直线到加速段
            correction_end_x = max(start_x + 1, self.correction_distance)
            for i in range(int(start_x), int(correction_end_x) + 1):
                self.planned_path.append((float(i), 0.0))
            rospy.loginfo("✅ 生成直线路径（无需纠偏）")
        
        # 🚀 阶段2：直线加速
        for i in range(int(correction_end_x) + 1, int(self.acceleration_length) + 1):
            self.planned_path.append((float(i), 0.0))
        
        # 🛑 阶段3：直线减速
        for i in range(int(self.acceleration_length) + 1, int(self.track_length) + 1):
            self.planned_path.append((float(i), 0.0))
        
        rospy.loginfo(f"🛣️ 路径重生成完成: 总共{len(self.planned_path)}个点")

    def generate_correction_curve(self, start_x, end_x, current_theta, current_y):
        """🎯 修正版纠偏曲线 - 强制向中心线收敛"""
        path_points = []
        L = end_x - start_x
        
        if L <= 0.1:
            return [(start_x, current_y)]
        
        rospy.loginfo(f"🔧 纠偏参数: 起点({start_x:.1f}, {current_y:.3f}), "
                     f"终点({end_x:.1f}, 0.0), 角度{math.degrees(current_theta):.1f}°")
        
        # 强制目标：在纠偏距离内，Y坐标必须线性回归到0
        # 提高路径点密度以获得更平滑的控制
        num_points = int(L * 5) + 1 
        for i in range(num_points):
            progress = float(i) / (num_points - 1) if num_points > 1 else 1.0
            x = start_x + progress * L
            
            # 线性插值Y坐标，确保路径向中心线收敛
            y = current_y * (1.0 - progress)
            
            path_points.append((x, y))
            
            # 调试输出关键点
            if i < 5 or i == num_points - 1:
                rospy.loginfo(f"纠偏点[{i}]: ({x:.2f}, {y:.3f})")
        
        return path_points

    def car_state_callback(self, msg):
        """🚗 车辆状态回调"""
        try:
            self.current_car_state = {
                'x': msg.car_state.x,
                'y': msg.car_state.y,
                'theta': msg.car_state.theta,
                'v_x': msg.car_state_dt.car_state_dt.x,
                'v_y': msg.car_state_dt.car_state_dt.y
            }
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"解析CarState失败: {e}")

    def emergency_safety_check(self, event):
        """🚨 紧急安全检查"""
        car_x = self.current_car_state['x']
        car_y = self.current_car_state['y']
        
        # 🚨 安全边界检查（3米赛道）
        if car_x >= 80.0 or abs(car_y) > 1.3:  # 降低到1.3米触发紧急停车
            if not self.emergency_stop_triggered:
                rospy.logfatal(f"🚨 紧急停车！位置({car_x:.1f}, {car_y:.1f})")
                self.emergency_stop_triggered = True

            self.current_target_speed = 0.0
            speed_msg = Float32()
            speed_msg.data = 0.0
            self.target_speed_pub.publish(speed_msg)

    def planning_loop(self, event):
        """🔄 主规划循环 - 添加实时路径重生成"""
        if self.emergency_stop_triggered:
            return

        if not self.planned_path:
            self.generate_optimal_path()  # 初始生成路径
            return
        
        # 检查是否需要重新生成路径
        if self.need_path_regeneration():
            self.generate_optimal_path()  # 重新生成路径
        
        self.check_correction_status()
        self.plan_speed()
        
        # 📡 发布
        self.publish_paths()
        self.publish_speeds()
        self.publish_status()
        
        # 📊 调试日志
        car_x = self.current_car_state['x']
        car_y = self.current_car_state['y']
        car_theta_deg = math.degrees(self.current_car_state['theta'])
        current_speed = math.sqrt(self.current_car_state['v_x']**2 + self.current_car_state['v_y']**2)
        
        rospy.loginfo_throttle(2.0, 
            f"🚗 状态: 位置({car_x:.1f}, {car_y:+.3f}), 角度{car_theta_deg:+.2f}°, "
            f"速度{current_speed:.1f}→{self.current_target_speed:.1f}m/s")

    def need_path_regeneration(self):
        """降低敏感度的触发条件"""
        car_y = self.current_car_state['y']
        car_theta_deg = abs(math.degrees(self.current_car_state['theta']))
        
        # ✅ 放宽触发条件，避免频繁重规划
        lateral_threshold = 0.4   # 0.4米偏离才重规划
        angle_threshold = 3.0     # 3.0度偏离才重规划
        
        result = abs(car_y) > lateral_threshold or car_theta_deg > angle_threshold
        if result:
            rospy.logwarn_throttle(2.0, 
                f"🔄 触发重生成: Y={car_y:.3f}m, θ={car_theta_deg:.2f}°")
        return result

    def check_correction_status(self):
        """🎯 检查纠偏状态"""
        car_x = self.current_car_state['x']
        car_y = self.current_car_state['y']
        car_theta_deg = math.degrees(self.current_car_state['theta'])
        
        # 更严格的纠偏完成条件
        if car_x >= self.correction_distance:
            self.correction_complete = True
        elif abs(car_theta_deg) < 0.5 and abs(car_y) < 0.2:  # 更严格的条件
            self.correction_complete = True
        else:
            self.correction_complete = False

    def plan_speed(self):
        """🚀 自适应速度规划 - 优化版"""
        car_x = self.current_car_state['x']
        car_y = self.current_car_state['y']
        car_theta_deg = abs(math.degrees(self.current_car_state['theta']))
        
        lateral_error = abs(car_y)
        angle_error = car_theta_deg
        
        # 🚨 紧急停车条件
        if car_x >= 78.0 or lateral_error > 1.2:
            self.current_target_speed = 0.0
            rospy.logwarn_throttle(1.0, f"🚨 紧急停车！X={car_x:.1f}, Y={car_y:.2f}")
            return
        
        # 🛑 减速阶段
        if car_x >= self.acceleration_length:
            distance_over = car_x - self.acceleration_length
            if distance_over <= 3.0:
                decel_factor = max(0.0, (3.0 - distance_over) / 3.0)
                self.current_target_speed = 3.0 * decel_factor
            else:
                self.current_target_speed = 0.0
            return
        
        # 🎯 自适应速度控制：基于偏离程度
        stage = ""
        if lateral_error > 0.8 or angle_error > 5.0:
            # 严重偏离：极低速纠偏
            self.current_target_speed = 1.0  # 降低到1.0m/s
            stage = f"🚨严重偏离"
        elif lateral_error > 0.4 or angle_error > 3.0:
            # 中等偏离：低速纠偏
            self.current_target_speed = 2.0  # 降低到2.0m/s
            stage = f"⚠️中等偏离"
        elif lateral_error > 0.2 or angle_error > 1.5:
            # 轻微偏离：谨慎行驶
            self.current_target_speed = 3.0
            stage = f"🔧轻微偏离"
        else:
            # 正常行驶：正常加速
            if car_x < 30:
                self.current_target_speed = 4.0
            elif car_x < 50:
                self.current_target_speed = 6.0
            else:
                self.current_target_speed = 8.0
            stage = f"🚀正常行驶"
        
        rospy.loginfo_throttle(3.0, 
            f"{stage}: 横向{lateral_error:.3f}m, 角度{angle_error:.2f}°, 速度{self.current_target_speed:.1f}m/s")

    def create_identity_quaternion(self):
        """🧭 创建单位四元数"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        return q

    def create_path_msg(self, path_points, frame_id="map"):
        """📍 创建Path消息"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = frame_id
        
        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = frame_id
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self.create_identity_quaternion()
            
            path_msg.poses.append(pose)
        
        return path_msg

    def publish_paths(self):
        """📡 发布路径"""
        if not self.planned_path:
            return
            
        path_msg = self.create_path_msg(self.planned_path)
        
        self.path_pub.publish(path_msg)
        self.planned_path_pub.publish(path_msg)
        self.planning_line_pub.publish(path_msg)
        
        self.publish_debug_markers()

    def publish_speeds(self):
        """📡 发布目标速度"""
        speed_msg = Float32()
        speed_msg.data = self.current_target_speed
        self.target_speed_pub.publish(speed_msg)

    def publish_status(self):
        """📡 发布状态信息"""
        car_x = self.current_car_state['x']
        car_y = self.current_car_state['y']
        
        if self.emergency_stop_triggered:
            status = f"🚨 紧急停车 - 位置({car_x:.1f}, {car_y:.1f})"
        elif car_x < self.correction_distance and not self.correction_complete:
            angle_deg = math.degrees(self.current_car_state['theta'])
            status = f"🔧 角度纠偏 - {car_x:.1f}m/{self.correction_distance}m, 角度{angle_deg:.2f}°, Y{car_y:+.3f}m"
        elif car_x < self.acceleration_length:
            progress = ((car_x - self.correction_distance) / (self.acceleration_length - self.correction_distance)) * 100
            status = f"🚀 直线加速 - {progress:.1f}%, 速度{self.current_target_speed:.1f}m/s"
        else:
            distance_over = car_x - self.acceleration_length
            status = f"🛑 制动阶段 - 超过{distance_over:.1f}m, 减速至{self.current_target_speed:.1f}m/s"
        
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def publish_debug_markers(self):
        """🎨 发布可视化标记"""
        if not self.planned_path:
            return
            
        markers = MarkerArray()
        car_x = self.current_car_state['x']
        
        # 🛣️ 路径点标记
        for i, (x, y) in enumerate(self.planned_path[::5]):  # 每5个点显示一个，避免卡顿
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation = self.create_identity_quaternion()
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # 根据阶段设置颜色
            if x < self.correction_distance and not self.correction_complete:
                marker.color.r = 1.0; marker.color.g = 0.6; marker.color.b = 0.0; marker.color.a = 0.9 # 橙色
            elif x < self.acceleration_length:
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.8 # 绿色
            else:
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8 # 红色
            
            markers.markers.append(marker)
        
        # 🎯 目标点标记
        current_speed = math.sqrt(self.current_car_state['v_x']**2 + self.current_car_state['v_y']**2)
        lookahead_distance = max(1.5, min(4.0, current_speed * 0.3))
        
        # 找到路径上最接近(car_x + lookahead_distance)的点
        target_x = car_x + lookahead_distance
        target_idx = 0
        min_dist = float('inf')
        for i, (px, py) in enumerate(self.planned_path):
            dist = abs(px - target_x)
            if dist < min_dist:
                min_dist = dist
                target_idx = i
            if px > target_x: # 优化：超过目标x后停止搜索
                break

        if target_idx < len(self.planned_path):
            target_marker = Marker()
            target_marker.header.frame_id = "map"
            target_marker.header.stamp = rospy.Time.now()
            target_marker.ns = "target_point"
            target_marker.id = 9999
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            target_marker.pose.position.x = self.planned_path[target_idx][0]
            target_marker.pose.position.y = self.planned_path[target_idx][1]
            target_marker.pose.position.z = 0.5
            target_marker.pose.orientation = self.create_identity_quaternion()
            
            target_marker.scale.x = 0.6
            target_marker.scale.y = 0.6
            target_marker.scale.z = 0.6
            
            target_marker.color.r = 0.0; target_marker.color.g = 0.0; target_marker.color.b = 1.0; target_marker.color.a = 1.0 # 蓝色
            
            markers.markers.append(target_marker)
            
            # 发布目标点
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = self.planned_path[target_idx][0]
            target_pose.pose.position.y = self.planned_path[target_idx][1]
            target_pose.pose.position.z = 0.5
            target_pose.pose.orientation = self.create_identity_quaternion()
            self.target_point_pub.publish(target_pose)
        
        self.markers_pub.publish(markers)

if __name__ == '__main__':
    try:
        planner = SimplePathPlanner()
        rospy.loginfo("🏁 智能路径规划器运行中...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🏁 路径规划器安全关闭")
    except Exception as e:
        rospy.logfatal(f"🚨 路径规划器异常: {str(e)}")