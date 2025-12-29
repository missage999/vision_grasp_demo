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
        self.controller = GraspController()
        
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