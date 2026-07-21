#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import yaml
import os
import math
import transforms3d

class StartupPoseLogger(Node):
    def __init__(self):
        super().__init__('startup_pose_logger')

        # 1. 声明参数 (参考点与保存路径)
        self.declare_parameter('ref_x', 0.0)
        self.declare_parameter('ref_y', 0.0)
        self.declare_parameter('ref_yaw', 0.0)
        self.declare_parameter('save_path', '/home/ek/RC2026/src/relocalization/config/startup_offset.yaml')
        self.declare_parameter('one_shot', True) 
        # True:默认只记录一次;False:一直记录

        # 2. 获取参数
        self.ref_x = self.get_parameter('ref_x').value
        self.ref_y = self.get_parameter('ref_y').value
        self.ref_yaw = self.get_parameter('ref_yaw').value
        self.save_path = self.get_parameter('save_path').value
        self.one_shot = self.get_parameter('one_shot').value

        self.has_logged = False

        # 3. 订阅 AMCL 定位结果
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # 4. 发布偏差值
        self.pub = self.create_publisher(Pose, '/startup_offset', 10)

        self.get_logger().info(f"偏差记录节点启动 | 参考原点: ({self.ref_x:.2f}, {self.ref_y:.2f})")

    def amcl_callback(self, msg):
        if self.one_shot and self.has_logged:
            return

        # --- 坐标计算 ---
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # 四元数 -> 欧拉角 Yaw
        _, _, curr_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

        # 计算偏差 (当前 - 参考)
        diff_x = curr_x - self.ref_x
        diff_y = curr_y - self.ref_y
        diff_yaw = curr_yaw - self.ref_yaw

        # 角度归一化 (-pi ~ pi)
        while diff_yaw > math.pi: diff_yaw -= 2*math.pi
        while diff_yaw < -math.pi: diff_yaw += 2*math.pi

        # --- 发布消息 ---
        out_msg = Pose()
        out_msg.position.x = diff_x
        out_msg.position.y = diff_y
        out_msg.orientation = self.yaw_to_quat(diff_yaw)
        self.pub.publish(out_msg)

        # --- 写入文件 ---
        self.write_yaml(diff_x, diff_y, diff_yaw)
        self.has_logged = True

    def write_yaml(self, x, y, yaw):
        data = {
            'startup_offset': {
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw)
            }
        }
        try:
            # 自动创建目录
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"定位成功! 偏差已写入: X={x:.3f}, Y={y:.3f}")
        except Exception as e:
            self.get_logger().error(f"写入文件失败: {e}")

    def yaw_to_quat(self, yaw):
        w, x, y, z = transforms3d.euler.euler2quat(0, 0, yaw)
        q = Pose().orientation
        q.w, q.x, q.y, q.z = w, x, y, z
        return q

def main(args=None):
    rclpy.init(args=args)
    node = StartupPoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()