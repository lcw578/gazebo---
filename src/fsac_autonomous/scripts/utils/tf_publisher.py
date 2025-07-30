#!/usr/bin/env python3
# filepath: src/fsac_autonomous/scripts/utils/tf_publisher.py
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher')
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 发布静态变换
        self.publish_static_transforms()
        
        # 定时发布动态变换
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_dynamic_transforms)
        
        rospy.loginfo("TF发布器启动完成")

    def publish_static_transforms(self):
        """发布静态坐标变换"""
        static_transforms = []
        
        # map -> odom 变换
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        self.static_broadcaster.sendTransform(static_transforms)
        rospy.loginfo("发布静态变换: map -> odom")

    def publish_dynamic_transforms(self, event):
        """发布动态坐标变换"""
        pass

if __name__ == '__main__':
    try:
        tf_pub = TFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass