#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化车辆控制模块 - 专门用于直线加速赛道
功能：Pure Pursuit + PID控制，生成控制线
作者：[你的姓名]
日期：[日期]
说明：直线加速赛道无障碍物，专注于路径跟踪和速度控制
"""
import rospy
import math
import numpy as np
from collections import deque
from fsd_common_msgs.msg import CarState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, Float32, Bool

class SimpleVehicleController:
    def __init__(self):
        rospy.init_node('simple_vehicle_controller', anonymous=True)
        
        # =====================================================
        # 车辆参数
        # =====================================================
        self.wheelbase = rospy.get_param('~wheelbase', 0.3302)  # racecar的实际轴距，不是2.65
        self.max_steering_angle = math.radians(25.0)  # 最大转向角(25度)
        self.max_speed = 15.0           # 最大速度(m/s)
        
        # =====================================================
        # Pure Pursuit 参数
        # =====================================================
        self.base_lookahead = 3.5       # 基础前瞻距离(m)
        self.speed_lookahead_gain = 0.3 # 速度增益
        self.min_lookahead = 2.0        # 最小前瞻距离(m)
        self.max_lookahead = 8.0        # 最大前瞻距离(m)
        self.lateral_error_gain = 1.5   # 🎯【新增】横向误差增益
        
        # 分阶段前瞻参数
        self.alignment_lookahead = 5.0  # 摆正阶段：大前瞻，稳定性优先
        self.tracking_lookahead = 3.5   # 跟踪阶段：正常前瞻
        
        # =====================================================
        # PID 速度控制参数
        # =====================================================
        self.speed_kp = 0.6             # 比例增益
        self.speed_ki = 0.15            # 积分增益
        self.speed_kd = 0.03            # 微分增益
        self.speed_integral_max = 3.0   # 积分限幅
        
        # =====================================================
        # 控制状态
        # =====================================================
        self.current_car_state = None
        self.planned_path = []
        self.target_speed = 0.0
        self.is_alignment_complete = False
        
        # 控制历史
        self.speed_error_history = deque(maxlen=10)
        self.speed_integral = 0.0
        
        # 控制输出
        self.current_steering_angle = 0.0
        self.current_speed_command = 0.0
        self.target_point = None
        
        # 控制轨迹记录（红色控制线）
        self.control_trajectory = deque(maxlen=150)  # 记录150个点
        
        # =====================================================
        # 订阅器
        # =====================================================
        self.state_sub = rospy.Subscriber(
            '/estimation/slam/state',
            CarState,
            self.car_state_callback,
            queue_size=10
        )
        
        self.path_sub = rospy.Subscriber(
            '/planning/planned_path',
            Path,
            self.path_callback,
            queue_size=10
        )
        
        self.target_speed_sub = rospy.Subscriber(
            '/planning/target_speed',
            Float32,
            self.target_speed_callback,
            queue_size=10
        )
        
        self.alignment_status_sub = rospy.Subscriber(
            '/planning/alignment_status',
            Bool,
            self.alignment_status_callback,
            queue_size=10
        )
        
        # =====================================================
        # 发布器
        # =====================================================
        # 控制指令
        self.cmd_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)
        
        # 控制线可视化（红色 - 实际控制轨迹）
        self.control_line_pub = rospy.Publisher('/control/control_line', MarkerArray, queue_size=10)
        
        # 目标点可视化（蓝色球）
        self.target_point_pub = rospy.Publisher('/control/target_point', PoseStamped, queue_size=10)
        
        # 控制状态
        self.control_status_pub = rospy.Publisher('/control/status', String, queue_size=10)
        
        # 调试信息
        self.steering_angle_pub = rospy.Publisher('/control/steering_angle', Float32, queue_size=10)
        self.control_debug_pub = rospy.Publisher('/control/debug_markers', MarkerArray, queue_size=10)
        
        # =====================================================
        # 定时器
        # =====================================================
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)     # 50Hz控制
        self.viz_timer = rospy.Timer(rospy.Duration(0.1), self.publish_visualization) # 10Hz可视化
        
        rospy.loginfo("=== 简化车辆控制模块启动 ===")
        rospy.loginfo(f"控制算法: Pure Pursuit + PID")
        rospy.loginfo(f"控制频率: 50Hz")
        rospy.loginfo(f"车辆参数: 轴距{self.wheelbase}m, 最大转向{math.degrees(self.max_steering_angle)}°")

    def car_state_callback(self, msg):
        """车辆状态回调"""
        self.current_car_state = {
            'x': msg.car_state.x,
            'y': msg.car_state.y,
            'theta': msg.car_state.theta,
            'vx': msg.car_state_dt.car_state_dt.x,
            'vy': msg.car_state_dt.car_state_dt.y,
            'speed': math.sqrt(msg.car_state_dt.car_state_dt.x**2 + msg.car_state_dt.car_state_dt.y**2),
            'timestamp': rospy.Time.now()
        }
        
        # 记录控制轨迹（用于红色控制线）
        current_pos = (self.current_car_state['x'], self.current_car_state['y'])
        
        # 每20cm记录一个点，避免轨迹过密
        if (len(self.control_trajectory) == 0 or 
            math.sqrt((current_pos[0] - self.control_trajectory[-1][0])**2 + 
                     (current_pos[1] - self.control_trajectory[-1][1])**2) > 0.2):
            self.control_trajectory.append(current_pos)

    def path_callback(self, msg):
        """规划路径回调"""
        self.planned_path = []
        for pose in msg.poses:
            self.planned_path.append((pose.pose.position.x, pose.pose.position.y))

    def target_speed_callback(self, msg):
        """目标速度回调"""
        self.target_speed = max(0.0, min(msg.data, self.max_speed))

    def alignment_status_callback(self, msg):
        """摆正状态回调"""
        prev_status = self.is_alignment_complete
        self.is_alignment_complete = msg.data
        
        if prev_status != self.is_alignment_complete and self.is_alignment_complete:
            rospy.loginfo("✓ 车头摆正完成，切换到中心线跟踪模式")

    def control_loop(self, event):
        """主控制循环 - 50Hz"""
        if not self.is_ready_for_control():
            return
        
        try:
            # 步骤1: Pure Pursuit 转向控制
            steering_angle = self.pure_pursuit_control()
            
            # 步骤2: PID 速度控制
            speed_command = self.pid_speed_control()
            
            # 步骤3: 发布控制指令
            self.publish_control_command(steering_angle, speed_command)
            
            # 步骤4: 更新状态
            self.current_steering_angle = steering_angle
            self.current_speed_command = speed_command
            
        except Exception as e:
            rospy.logwarn(f"控制异常: {e}")
            # 保持当前状态，不做急剧变化
            self.publish_control_command(self.current_steering_angle, 0.0)

    def is_ready_for_control(self):
        """检查是否准备好进行控制"""
        if self.current_car_state is None:
            return False
        
        if len(self.planned_path) < 2:
            rospy.logdebug_throttle(2.0, "等待规划路径...")
            return False
        
        return True

    def pure_pursuit_control(self):
        """修正的横向偏差控制 - 按照ROS标准约定"""
        # 获取当前状态
        lateral_error = self.current_car_state['y']      # y偏差（目标是y=0）
        angular_error = self.current_car_state['theta']  # 偏航角偏差（目标是theta=0）
        
        # PID控制器参数
        kp_lateral = 2.0    # 横向偏差增益
        kp_angular = 1.2    # 角度偏差增益
        
        # 🎯 ROS标准控制逻辑：
        # 横向控制：
        #   - y > 0 (车在左侧) → 需要右转 (负转向角)
        #   - y < 0 (车在右侧) → 需要左转 (正转向角)
        # 角度控制：
        #   - theta > 0 (车头偏左) → 需要右转纠正 (负转向角)
        #   - theta < 0 (车头偏右) → 需要左转纠正 (正转向角)
        
        # 修正的控制公式：
        steering_angle = (kp_lateral * lateral_error + kp_angular * angular_error)
        
        # 限制转向角
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        # 设置虚拟目标点用于可视化
        car_x = self.current_car_state['x']
        target_y = 0.0  # 目标始终是中心线
        self.target_point = (car_x + 3.0, target_y)  # 前方3米处的中心线点
        
        # 🔍 详细调试信息
        y_status = "左侧" if lateral_error > 0 else "右侧" if lateral_error < 0 else "中心"
        theta_status = "偏左" if angular_error > 0 else "偏右" if angular_error < 0 else "正向"
        steer_direction = "左转" if steering_angle > 0 else "右转" if steering_angle < 0 else "直行"
        
        rospy.loginfo_throttle(1.0, 
            f"🎯 ROS标准控制: Y={lateral_error:+.4f}m({y_status}), "
            f"θ={math.degrees(angular_error):+.2f}°({theta_status}), "
            f"转向={math.degrees(steering_angle):+.2f}°({steer_direction})")
        
        return steering_angle

    def calculate_lookahead_distance(self):
        """保持原有接口，但不再使用Pure Pursuit"""
        # 这个方法现在不被使用，但保留以免影响其他代码
        return self.base_lookahead

    def find_target_point(self, lookahead_distance):
        """保持原有接口，但简化实现"""
        # 直接返回前方中心线上的点
        car_x = self.current_car_state['x']
        target_x = car_x + lookahead_distance
        target_y = 0.0  # 始终瞄准中心线
        return (target_x, target_y)

    def calculate_steering_angle(self, target_point):
        """保持原有接口，但不再使用Pure Pursuit公式"""
        # 这个方法现在在pure_pursuit_control中直接实现
        # 保留此方法以免影响其他代码
        return 0.0

    def pid_speed_control(self):
        """PID速度控制"""
        current_speed = self.current_car_state['speed']
        speed_error = self.target_speed - current_speed
        
        # 积分项
        self.speed_integral += speed_error * 0.02  # dt = 0.02s (50Hz)
        self.speed_integral = max(-self.speed_integral_max, 
                                min(self.speed_integral_max, self.speed_integral))
        
        # 微分项
        self.speed_error_history.append(speed_error)
        if len(self.speed_error_history) >= 2:
            speed_derivative = (self.speed_error_history[-1] - self.speed_error_history[-2]) / 0.02
        else:
            speed_derivative = 0.0
        
        # PID输出
        speed_command = (self.speed_kp * speed_error + 
                        self.speed_ki * self.speed_integral + 
                        self.speed_kd * speed_derivative)
        
        # 简单的速度限制
        speed_command = max(0.0, min(self.max_speed, speed_command))
        
        return speed_command

    def publish_control_command(self, steering_angle, speed_command):
        """发布控制指令"""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        
        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = speed_command
        
        self.cmd_pub.publish(cmd)
        
        # 发布转向角调试信息
        steering_msg = Float32()
        steering_msg.data = math.degrees(steering_angle)
        self.steering_angle_pub.publish(steering_msg)

    def publish_visualization(self, event):
        """发布可视化信息"""
        # 发布控制线（红色）
        self.publish_control_line()
        
        # 发布目标点（蓝色球）
        self.publish_target_point_marker()
        
        # 发布控制状态
        self.publish_control_status()
        
        # 发布调试标记
        self.publish_debug_markers()

    def publish_control_line(self):
        """发布控制线 - 红色实际行驶轨迹"""
        if len(self.control_trajectory) < 2:
            return
        
        marker_array = MarkerArray()
        
        # 控制轨迹线（红色）
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "control_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # 红色线条
        line_marker.scale.x = 0.2       # 线宽20cm
        line_marker.color.r = 1.0       # 红色
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.9
        
        # 添加轨迹点
        for point in self.control_trajectory:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.05  # 稍微抬高
            line_marker.points.append(p)
        
        line_marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(line_marker)
        
        self.control_line_pub.publish(marker_array)

    def publish_target_point_marker(self):
        """发布目标点标记 - 蓝色球体"""
        if self.target_point is None:
            return
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = self.target_point[0]
        pose.pose.position.y = self.target_point[1]
        pose.pose.position.z = 0.5
        pose.pose.orientation.w = 1.0
        
        self.target_point_pub.publish(pose)

    def publish_control_status(self):
        """发布控制状态"""
        if self.current_car_state is None:
            return
        
        mode = "摆正中" if not self.is_alignment_complete else "跟踪中"
        car_x = self.current_car_state['x']
        current_speed = self.current_car_state['speed']
        
        status = f"控制模式: {mode} | " \
                f"位置: {car_x:.1f}m | " \
                f"速度: {current_speed:.1f}/{self.target_speed:.1f}m/s | " \
                f"转向: {math.degrees(self.current_steering_angle):.1f}° | " \
                f"路径点: {len(self.planned_path)}"
        
        msg = String()
        msg.data = status
        self.control_status_pub.publish(msg)

    def publish_debug_markers(self):
        """发布调试标记"""
        if self.current_car_state is None or self.target_point is None:
            return
        
        marker_array = MarkerArray()
        
        # 目标点球体（蓝色）
        target_sphere = Marker()
        target_sphere.header.frame_id = "map"
        target_sphere.header.stamp = rospy.Time.now()
        target_sphere.ns = "control_debug"
        target_sphere.id = 0
        target_sphere.type = Marker.SPHERE
        target_sphere.action = Marker.ADD
        
        target_sphere.pose.position.x = self.target_point[0]
        target_sphere.pose.position.y = self.target_point[1]
        target_sphere.pose.position.z = 0.3
        target_sphere.pose.orientation.w = 1.0
        
        target_sphere.scale.x = 0.6
        target_sphere.scale.y = 0.6
        target_sphere.scale.z = 0.6
        
        target_sphere.color.r = 0.0
        target_sphere.color.g = 0.0
        target_sphere.color.b = 1.0  # 蓝色
        target_sphere.color.a = 0.7
        
        target_sphere.lifetime = rospy.Duration(0.3)
        marker_array.markers.append(target_sphere)
        
        # 连接线：车辆到目标点（虚线效果）
        if abs(self.current_steering_angle) > 0.02:  # 只在转向时显示
            connection_line = Marker()
            connection_line.header.frame_id = "map"
            connection_line.header.stamp = rospy.Time.now()
            connection_line.ns = "control_debug"
            connection_line.id = 1
            connection_line.type = Marker.LINE_STRIP
            connection_line.action = Marker.ADD
            
            # 起点：车辆位置
            start_point = Point()
            start_point.x = self.current_car_state['x']
            start_point.y = self.current_car_state['y']
            start_point.z = 0.2
            
            # 终点：目标点
            end_point = Point()
            end_point.x = self.target_point[0]
            end_point.y = self.target_point[1]
            end_point.z = 0.2
            
            connection_line.points = [start_point, end_point]
            
            connection_line.scale.x = 0.05  # 细线
            connection_line.color.r = 1.0
            connection_line.color.g = 1.0
            connection_line.color.b = 0.0  # 黄色
            connection_line.color.a = 0.5  # 半透明
            
            connection_line.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(connection_line)
        
        self.control_debug_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        controller = SimpleVehicleController()
        rospy.loginfo("简化车辆控制模块运行中...")
        rospy.loginfo("🚗 控制策略：Pure Pursuit转向 + PID速度控制")
        rospy.loginfo("🔴 红色线：实际控制轨迹")
        rospy.loginfo("🔵 蓝色球：当前目标点")
        rospy.loginfo("🟡 黄色线：车辆到目标点连线")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("简化车辆控制模块正常退出")
    except Exception as e:
        rospy.logerr(f"简化车辆控制模块错误: {e}")
        import traceback
        traceback.print_exc()