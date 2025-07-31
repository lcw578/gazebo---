#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FSAC车辆生成器 - 随机偏航角初始化
功能：在起点前0.3m生成车辆，随机偏航角-3°到+3°
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

def spawn_racecar():
    rospy.init_node('generate_yaw_and_spawn', anonymous=True)
    
    # 等待Gazebo服务
    rospy.loginfo("等待Gazebo服务启动...")
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # 尝试删除可能存在的旧模型
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=2.0)
        delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_srv('racecar')
        rospy.loginfo("删除已存在的车辆模型")
        rospy.sleep(1.0)  # 等待删除完成
    except:
        rospy.loginfo("没有找到已存在的车辆模型")
    
    # 等待robot_description参数
    rospy.loginfo("等待robot_description参数...")
    while not rospy.has_param('/robot_description') and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    robot_description = rospy.get_param('/robot_description')
    rospy.loginfo("获取到robot_description")
    
    # 生成随机偏航角 (-3度到+3度)
    yaw_degrees = random.uniform(-3.0, 3.0)
    yaw_radians = math.radians(yaw_degrees)
    
    # 车辆参数（根据你的URDF调整）
    car_length = 2.2  # 车辆长度，需要根据实际URDF确认
    car_front_to_base_link = car_length / 2  # 车头到base_link的距离
    
    # 计算base_link位置：车头距起点0.3米
    base_link_x = -0.3 - car_front_to_base_link  # = -0.3 - 0.25 = -0.55
    
    rospy.loginfo(f"🎯 FSAC规则初始化:")
    rospy.loginfo(f"   车头距起点: 0.3米")
    rospy.loginfo(f"   base_link位置: x={base_link_x:.3f}米")
    rospy.loginfo(f"   随机偏航角: {yaw_degrees:.2f}度")
    
    # 设置初始位置
    initial_pose = Pose()
    initial_pose.position.x = base_link_x  # 调整后的位置
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.1
    initial_pose.orientation = quaternion_from_euler(0, 0, yaw_radians)
    
    # 生成车辆
    try:
        response = spawn_model_srv(
            model_name='racecar',
            model_xml=robot_description,
            robot_namespace='',
            initial_pose=initial_pose,
            reference_frame='world'
        )
        
        if response.success:
            rospy.loginfo("✅ 车辆生成成功!")
            rospy.sleep(2.0)  # 等待车辆稳定
            
            # 再次设置车辆位置和姿态，确保生效
            model_state = ModelState()
            model_state.model_name = 'racecar'
            model_state.pose = initial_pose
            model_state.twist = Twist()  # 零速度
            model_state.reference_frame = 'world'
            
            try:
                set_model_state_srv(model_state)
                rospy.loginfo(f"✅ 车辆位置设置完成:")
                rospy.loginfo(f"   位置: x={initial_pose.position.x:.3f}m, y={initial_pose.position.y:.3f}m")
                rospy.loginfo(f"   偏航角: {yaw_degrees:.2f}度")
                rospy.loginfo(f"   激光雷达高度: 0.2米 (需检查URDF)")
                
            except Exception as e:
                rospy.logwarn(f"设置车辆状态失败: {e}")
                
        else:
            rospy.logerr(f"❌ 车辆生成失败: {response.status_message}")
            
    except Exception as e:
        rospy.logerr(f"❌ 调用生成服务失败: {e}")
    
    # 发布初始参数到参数服务器
    rospy.set_param('/fsac/initial_yaw_degrees', yaw_degrees)
    rospy.set_param('/fsac/initial_x', initial_pose.position.x)
    rospy.set_param('/fsac/initial_y', initial_pose.position.y)
    rospy.loginfo(f"✅ 设置初始偏角参数: {yaw_degrees:.2f}°")
if __name__ == '__main__':
    try:
        spawn_racecar()
    except rospy.ROSInterruptException:
        rospy.loginfo("车辆生成器退出")
    except Exception as e:
        rospy.logerr(f"车辆生成器出错: {e}")