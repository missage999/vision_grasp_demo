#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
import tf2_ros
import numpy as np

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # 参数：物体高度（桌面到物体中心的距离）
        self.declare_parameter('object_height', 0.05)  # 5cm高的方块，中心
        self.declare_parameter('preset_depth', 1.293)
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('target_frame', 'base_link')
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 相机内参（从CameraInfo话题获取）
        self.camera_info = None
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )
        
        # 订阅物体检测结果
        self.create_subscription(
            PoseStamped, '/detection/object_pose_pixel', self.pixel_pose_callback, 10
        )
        
        # 发布3D位姿
        self.pose_pub = self.create_publisher(PoseStamped, '/detection/object_pose_3d', 10)
        
        self.get_logger().info('PoseEstimator节点已启动 - 等待相机参数和物体像素坐标...')
    
    def camera_info_callback(self, msg):
        """缓存相机内参"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'相机参数已获取: fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}')
    
    def pixel_pose_callback(self, msg):
        """核心：像素坐标 → 3D位姿"""
        if self.camera_info is None:
            self.get_logger().warn('等待相机参数中...')
            return
        
        # 1. 提取像素坐标
        pixel_u = msg.pose.position.x  # 复用x,y字段存储像素坐标
        pixel_v = msg.pose.position.y
        
        # 2. 相机内参
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        
        # 3. 预设深度（关键简化：物体在桌面，高度已知）
        object_height = self.get_parameter('object_height').value
        # 相机离地高度约0.8m，物体高度0.05m，所以Zc≈0.75m
        Zc = self.get_parameter('preset_depth').value
        
        # 4. 像素 → 相机坐标系
        Xc = (pixel_u - cx) * Zc / fx
        Yc = (pixel_v - cy) * Zc / fy
        
        self.get_logger().debug(f'相机坐标: ({Xc:.3f}, {Yc:.3f}, {Zc:.3f})')
        
        # 5. 相机坐标系 → base_link
        try:
            # 获取 TF: base_link ← camera_optical_frame
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('target_frame').value,
                self.get_parameter('camera_frame').value,
                rclpy.time.Time()
            )
            
            # 提取旋转和平移
            t = transform.transform.translation
            q = transform.transform.rotation
            
            # 四元数 → 旋转矩阵
            R = self.quaternion_to_rotation_matrix([q.x, q.y, q.z, q.w])
            
            # 应用变换: P_base = R * P_camera + t
            P_camera = np.array([Xc, Yc, Zc])
            P_base = R @ P_camera + np.array([t.x, t.y, t.z])
            
            # red_box真实坐标 (根据启动文件配置)
            real_x, real_y, real_z = 0.8, 0.3, 0.05
            
            # 计算误差
            error_x = abs(P_base[0] - real_x)
            error_y = abs(P_base[1] - real_y)
            error_z = abs(P_base[2] - real_z)
            
            self.get_logger().info(
                f'物体在base_link下的位姿: x={P_base[0]:.3f}, y={P_base[1]:.3f}, z={P_base[2]:.3f} | '
                f'真实坐标: x={real_x:.3f}, y={real_y:.3f}, z={real_z:.3f} | '
                f'误差: x={error_x:.3f}, y={error_y:.3f}, z={error_z:.3f}'
            )
            
            # 6. 发布PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.get_parameter('target_frame').value
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            pose_msg.pose.position.x = P_base[0]
            pose_msg.pose.position.y = P_base[1]
            pose_msg.pose.position.z = object_height  # 物体中心高度
            
            # 朝向：默认朝上（可根据物体方向调整）
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(pose_msg)
            
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'TF变换失败: {str(e)}')
            self.get_logger().warn('请确认: 1) URDF中定义了camera_link_optical 2) robot_state_publisher已运行')
    
    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """四元数 → 3x3旋转矩阵"""
        x, y, z, w = q
        
        # 归一化
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # 计算旋转矩阵
        R = np.array([
            [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        
        return R


def main(args=None):
    rclpy.init(args=args)
    estimator = PoseEstimator()
    
    try:
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        estimator.get_logger().info('节点已停止')
    finally:
        estimator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()