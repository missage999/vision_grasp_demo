#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # 参数配置
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_contour_area', 300)  # 最小轮廓面积（过滤噪点）
        self.declare_parameter('aspect_ratio_tolerance', 0.3)  # 长宽比容差
        
        # 性能优化：将get_color_range()移到__init__中，避免每帧重复创建numpy数组
        # 初始化颜色范围（性能优化：避免每帧重复创建numpy数组）
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 120, 80]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 80, 50]),  # 红色跨区间
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 150, 0]),
                'upper': np.array([140, 255, 255])
            }
        }
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 创建发布者：发布像素坐标
        self.pixel_pose_pub = self.create_publisher(PoseStamped, '/detection/object_pose_pixel', 10)
        
        # 订阅摄像头图像
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # 发布调试图像（带检测框）
        self.debug_pub = self.create_publisher(Image, '/debug/detection', 10)
        
        self.get_logger().info('ObjectDetector节点已启动 - 等待图像输入...')
    
    def get_color_range(self, color_name):
        """获取HSV颜色范围（ROS参数化版本）"""
        # 注意：Gazebo仿真光照稳定，真实场景需动态调整
        return self.color_ranges.get(color_name, self.color_ranges['red'])
    
    def image_callback(self, msg):
        """主处理循环：每帧图像调用一次"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 识别物体
            detection = self.detect_object(cv_image)
            
            if detection:
                # 绘制检测框
                self.draw_detection(cv_image, detection)
                
                # 输出结果
                self.get_logger().info(
                    f"检测到物体: 中心={detection['center']}, "
                    f"尺寸={detection['size']}, 置信度={detection['confidence']:.2f}"
                )
                
                # 新增：发布像素坐标
                pixel_msg = PoseStamped()
                pixel_msg.header.frame_id = 'camera_optical_frame'
                pixel_msg.header.stamp = self.get_clock().now().to_msg()
                pixel_msg.pose.position.x = float(detection['center'][0])
                pixel_msg.pose.position.y = float(detection['center'][1])
                pixel_msg.pose.position.z = 0.0  # 像素坐标无深度
                
                self.pixel_pose_pub.publish(pixel_msg)
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'图像处理失败: {str(e)}')
    
    def detect_object(self, image):
        """
        核心识别算法：颜色阈值 + 几何过滤
        返回值: dict或None
        """
        # 1. 颜色空间转换
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 2. 颜色阈值分割
        color_name = self.get_parameter('target_color').value
        ranges = self.get_color_range(color_name)
        
        if color_name == 'red':
            # 红色需要合并两个区间
            mask1 = cv2.inRange(hsv, ranges['lower1'], ranges['upper1'])
            mask2 = cv2.inRange(hsv, ranges['lower2'], ranges['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, ranges['lower'], ranges['upper'])
        
        # 3. 形态学去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算填洞
        
        # 4. 轮廓检测
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        if not contours:
            return None
        
        # 5. 选择最大轮廓（假设目标最近/最大）
        main_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(main_contour)
        
        # 6. 面积过滤（去掉小噪点）
        min_area = self.get_parameter('min_contour_area').value
        if area < min_area:
            return None
        
        # 7. 形状验证（长宽比）
        x, y, w, h = cv2.boundingRect(main_contour)
        aspect_ratio = w / h if h > 0 else 0
        tolerance = self.get_parameter('aspect_ratio_tolerance').value
        
        if not (1 - tolerance < aspect_ratio < 1 + tolerance):
            # 不是正方形
            return None
        
        # 8. 计算置信度（面积越大越可信）
        confidence = min(area / 2000.0, 1.0)  # 归一化
        
        return {
            "center": (x + w//2, y + h//2),
            "size": (w, h),
            "color": color_name,
            "confidence": confidence,
            "contour": main_contour
        }
    
    def draw_detection(self, image, detection):
        """在图像上绘制检测结果"""
        center = detection['center']
        size = detection['size']
        contour = detection['contour']
        
        # 绘制轮廓
        cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)
        
        # 绘制中心点
        cv2.circle(image, center, 5, (255, 0, 0), -1)  # 改为蓝色，提高在红色物体上的可见性
        
        # 绘制信息文本
        text = f"Color: {detection['color']} | Conf: {detection['confidence']:.2f}"
        cv2.putText(image, text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('节点已停止')
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()