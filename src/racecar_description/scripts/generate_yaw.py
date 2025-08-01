#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
差速驱动小车生成器 - 随机偏航角初始化 🚗
功能：生成差速驱动小车，随机偏航角-3°到+3°
"""
import rospy
import random
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Twist

def quaternion_from_euler(roll, pitch, yaw):
    """从欧拉角转换为四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    return q

def load_urdf_model():
    """加载差速驱动小车URDF模型"""
    urdf_file = "/home/slz/catkin_ws/src/racecar_description/urdf/test_car.urdf.xacro"
    
    try:
        import subprocess
        result = subprocess.run(['xacro', urdf_file], 
                              capture_output=True, text=True, check=True)
        robot_description = result.stdout
        
        # 🔧 关键修复：将robot_description发布到参数服务器
        rospy.set_param('/robot_description', robot_description)
        rospy.loginfo("✅ 成功加载差速驱动小车URDF模型并发布到参数服务器")
        
        return robot_description
    except Exception as e:
        rospy.logerr(f"❌ 加载URDF模型失败: {e}")
        return None

def spawn_diff_drive_robot():
    rospy.init_node('spawn_diff_drive_robot', anonymous=True)
    
    # 等待Gazebo服务
    rospy.loginfo("🚗 等待Gazebo服务启动...")
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # 删除可能存在的旧模型
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=2.0)
        delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_srv('diff_drive_robot')
        rospy.loginfo("🗑️ 删除已存在的小车模型")
        rospy.sleep(1.0)
    except:
        rospy.loginfo("🆕 没有找到已存在的小车模型")
    
    # 加载URDF模型
    robot_description = load_urdf_model()
    if robot_description is None:
        rospy.logfatal("❌ 无法加载机器人模型，退出")
        return
    
    # 生成随机偏航角 (-3度到+3度)
    yaw_degrees = random.uniform(-3.0, 3.0)
    yaw_radians = math.radians(yaw_degrees)
    
    # 🚗 计算初始位置（符合新的车辆尺寸）
    car_length = 0.5      # 修改为实际车长
    car_front_to_base_link = car_length / 2
    base_link_x = -0.3 - car_front_to_base_link
    
    rospy.loginfo(f"🎯 差速驱动小车初始化:")
    rospy.loginfo(f"   车头距起点: 0.3米")
    rospy.loginfo(f"   base_link位置: x={base_link_x:.3f}米")
    rospy.loginfo(f"   随机偏航角: {yaw_degrees:.2f}度")
    rospy.loginfo(f"   轮距: 0.2米")          # 更新轮距
    rospy.loginfo(f"   轮径: 0.1米")          # 更新轮径
    rospy.loginfo(f"   车身尺寸: 0.5×0.25×0.15米")
    
    # 设置初始位置
    initial_pose = Pose()
    initial_pose.position.x = base_link_x
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.0  # 轮子接触地面
    initial_pose.orientation = quaternion_from_euler(0, 0, yaw_radians)
    
    # 生成车辆
    try:
        response = spawn_model_srv(
            model_name='diff_drive_robot',
            model_xml=robot_description,
            robot_namespace='',
            initial_pose=initial_pose,
            reference_frame='world'
        )
        
        if response.success:
            rospy.loginfo("✅ 差速驱动小车生成成功!")
            rospy.sleep(2.0)  # 等待车辆稳定
            
            # 确保车辆位置正确
            model_state = ModelState()
            model_state.model_name = 'diff_drive_robot'
            model_state.pose = initial_pose
            model_state.twist = Twist()  # 零速度
            model_state.reference_frame = 'world'
            
            try:
                set_model_state_srv(model_state)
                rospy.loginfo(f"✅ 小车位置设置完成:")
                rospy.loginfo(f"   位置: x={initial_pose.position.x:.3f}m, y={initial_pose.position.y:.3f}m")
                rospy.loginfo(f"   偏航角: {yaw_degrees:.2f}度")
                rospy.loginfo(f"   控制话题: /cmd_vel")
                rospy.loginfo(f"   里程计话题: /odom")
                
            except Exception as e:
                rospy.logwarn(f"⚠️ 设置车辆状态失败: {e}")
                
        else:
            rospy.logerr(f"❌ 小车生成失败: {response.status_message}")
            
    except Exception as e:
        rospy.logerr(f"❌ 调用生成服务失败: {e}")
    
    # 🔧 更新参数服务器的车辆参数
    rospy.set_param('/diff_drive/initial_yaw_degrees', yaw_degrees)
    rospy.set_param('/diff_drive/initial_x', initial_pose.position.x)
    rospy.set_param('/diff_drive/initial_y', initial_pose.position.y)
    rospy.set_param('/diff_drive/wheel_separation', 0.2)    # 更新轮距
    rospy.set_param('/diff_drive/wheel_diameter', 0.1)      # 更新轮径
    rospy.set_param('/diff_drive/car_length', 0.5)          # 车长
    rospy.set_param('/diff_drive/car_width', 0.25)          # 车宽
    
    rospy.loginfo(f"📊 差速驱动参数已设置到参数服务器")

if __name__ == '__main__':
    try:
        spawn_diff_drive_robot()
    except rospy.ROSInterruptException:
        rospy.loginfo("🚗 差速驱动小车生成器退出")
    except Exception as e:
        rospy.logerr(f"💥 差速驱动小车生成器出错: {e}")