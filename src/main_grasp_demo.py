#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GraspDemo(Node):
    def __init__(self):
        super().__init__('grasp_demo')
        
        # 订阅3D位姿
        self.object_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped, '/detection/object_pose_3d', self.pose_callback, 10
        )
        
        # 创建GraspController客户端（简化：直接实例化）
        from grasp_controller import GraspController
        
        # 加载配置文件参数
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('pre_grasp_height', 0.2)
        self.declare_parameter('grasp_height', 0.1)
        self.declare_parameter('velocity_scaling_factor', 0.3)
        self.declare_parameter('acceleration_scaling_factor', 0.3)
        
        # 创建 GraspController 实例
        self.controller = GraspController()
        
        # 设置参数（如果配置文件加载了，这些值会被覆盖）
        self.controller.set_parameters([
            rclpy.parameter.Parameter('planning_group', rclpy.Parameter.Type.STRING, self.get_parameter('planning_group').value),
            rclpy.parameter.Parameter('pre_grasp_height', rclpy.Parameter.Type.DOUBLE, self.get_parameter('pre_grasp_height').value),
            rclpy.parameter.Parameter('grasp_height', rclpy.Parameter.Type.DOUBLE, self.get_parameter('grasp_height').value),
            rclpy.parameter.Parameter('velocity_scaling_factor', rclpy.Parameter.Type.DOUBLE, self.get_parameter('velocity_scaling_factor').value),
            rclpy.parameter.Parameter('acceleration_scaling_factor', rclpy.Parameter.Type.DOUBLE, self.get_parameter('acceleration_scaling_factor').value),
        ])
        
        self.get_logger().info('GraspDemo主循环已启动 - 等待物体位姿...')
    
    def pose_callback(self, msg):
        self.object_pose = msg
        self.get_logger().info(f'接收到物体位姿: {msg.pose.position}')
    
    def run(self):
        """主循环：识别 → 规划 → 抓取"""
        while rclpy.ok() and self.object_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('等待物体检测...')
            time.sleep(1.0)
        
        if self.object_pose:
            success = self.controller.grasp_object(self.object_pose)
            if success:
                self.get_logger().info('🎉 抓取成功！')
                # 移动到放置位置（示例）
                place_pose = PoseStamped()
                place_pose.header.frame_id = "base_link"
                place_pose.pose.position.x = 0.3
                place_pose.pose.position.y = -0.3
                place_pose.pose.position.z = 0.05
                self.controller.grasp_object(place_pose)
            else:
                self.get_logger().error('❌ 抓取失败')
        
        self.controller.return_home()


def main(args=None):
    rclpy.init(args=args)
    demo = GraspDemo()
    
    try:
        demo.run()
    except KeyboardInterrupt:
        demo.get_logger().info('演示已停止')
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()