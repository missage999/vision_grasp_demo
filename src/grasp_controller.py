#!/usr/bin/env python3
"""
GraspController for ROS2 Humble
使用 MoveGroup Action 接口（比服务更可靠）
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import (
    CollisionObject, 
    PlanningScene, 
    Constraints, 
    OrientationConstraint,
    PositionConstraint
)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import time


class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')
        
        # 参数
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('pre_grasp_height', 0.2)
        self.declare_parameter('grasp_height', 0.1)
        
        self.planning_group = self.get_parameter('planning_group').value
        
        # 使用 Action 接口（正确名称: /move_action）
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info(f'等待 MoveGroup Action: /move_action...')
        timeout_count = 0
        max_timeout = 30
        
        while not self.move_action_client.wait_for_server(timeout_sec=1.0):
            timeout_count += 1
            if timeout_count % 5 == 0:
                self.get_logger().info(f'继续等待 MoveGroup Action... ({timeout_count}s)')
            if timeout_count >= max_timeout:
                self.get_logger().error(f'等待 MoveGroup Action 超时 ({max_timeout}s)')
                raise RuntimeError(f'MoveGroup Action /move_action 不可用')
        
        self.get_logger().info('MoveGroup Action 已就绪')
        
        # 创建规划场景发布者
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        self.get_logger().info(f'GraspController已启动 - 规划组: {self.planning_group}')
        
        # 等待一下让服务完全就绪
        time.sleep(0.5)
        self.add_collision_objects()
    
    def add_collision_objects(self):
        """添加碰撞体：地面"""
        self.get_logger().info('添加碰撞场景...')
        
        ground = CollisionObject()
        ground.id = "ground"
        ground.header.frame_id = "base_link"
        ground.operation = CollisionObject.ADD
        
        ground_pose = Pose()
        ground_pose.position.z = -0.01
        
        ground.primitive_poses = [ground_pose]
        ground.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0, 3.0, 0.01])]
        
        self._publish_planning_scene([ground])
        
        time.sleep(0.5)
        self.get_logger().info('碰撞场景添加完成')
    
    def _publish_planning_scene(self, collision_objects):
        """发布规划场景"""
        scene = PlanningScene()
        scene.world.collision_objects = collision_objects
        scene.is_diff = True
        self.scene_pub.publish(scene)
    
    def plan_to_pose(self, target_pose, constraints=None):
        """通用规划函数 - 使用 Action 接口"""
        self.get_logger().info(f'规划目标位姿...')
        
        # 创建 MoveGroup Action 目标
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.allowed_planning_time = 10.0  # ROS2 MoveIt2 中正确的属性名
        
        # 创建位姿约束
        pose_constraint = Constraints()
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = "tool0"
        position_constraint.constraint_region.primitive_poses = [target_pose.pose]
        position_constraint.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        ]
        position_constraint.weight = 1.0
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = "tool0"
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        if constraints and hasattr(constraints, 'orientation_constraints') and constraints.orientation_constraints:
            orientation_constraint = constraints.orientation_constraints[0]
        
        pose_constraint.position_constraints = [position_constraint]
        pose_constraint.orientation_constraints = [orientation_constraint]
        
        goal.request.goal_constraints = [pose_constraint]
        
        # 设置开始状态（使用当前状态）
        goal.request.start_state.is_diff = True
        
        self.get_logger().info('发送规划请求...')
        
        # 发送 Action 目标
        future = self.move_action_client.send_goal_async(goal)
        
        try:
            # 等待目标被接受
            start_time = time.time()
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > 15.0:
                    self.get_logger().error('发送规划目标超时')
                    return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('规划目标被拒绝')
                return False
            
            self.get_logger().info('规划目标已接受，等待结果...')
            
            # 获取结果
            result_future = goal_handle.get_result_async()
            
            start_time = time.time()
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > 15.0:
                    self.get_logger().error('等待规划结果超时')
                    return False
            
            result = result_future.result()
            
            if result and result.result.error_code.val == 1:
                self.get_logger().info('规划成功!')
                return True
            else:
                error_code = result.result.error_code.val if result else 'None'
                self.get_logger().error(f'规划失败: 错误码={error_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'规划异常: {str(e)}')
            return False
    
    def grasp_object(self, object_pose: PoseStamped):
        """主抓取流程"""
        self.get_logger().info('=== 开始抓取流程 ===')
        
        if object_pose.header.frame_id == '':
            object_pose.header.frame_id = 'base_link'
        
        # 1. 移动到预抓取位置
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = "base_link"
        pre_grasp_pose.pose.position.x = object_pose.pose.position.x
        pre_grasp_pose.pose.position.y = object_pose.pose.position.y
        pre_grasp_pose.pose.position.z = object_pose.pose.position.z + self.get_parameter('pre_grasp_height').value
        
        pre_grasp_pose.pose.orientation.x = 0.0
        pre_grasp_pose.pose.orientation.y = 0.707
        pre_grasp_pose.pose.orientation.z = 0.0
        pre_grasp_pose.pose.orientation.w = 0.707
        
        self.get_logger().info('步骤1: 移动到预抓取点')
        if not self.plan_to_pose(pre_grasp_pose):
            self.get_logger().error('预抓取规划失败')
            return False
        
        # 2. 设置方向约束
        constraints = Constraints()
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "tool0"
        orientation_constraint.orientation = pre_grasp_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints = [orientation_constraint]
        
        # 3. 接近物体
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.pose.position.x = object_pose.pose.position.x
        grasp_pose.pose.position.y = object_pose.pose.position.y
        grasp_pose.pose.position.z = object_pose.pose.position.z + self.get_parameter('grasp_height').value
        grasp_pose.pose.orientation = pre_grasp_pose.pose.orientation
        
        self.get_logger().info('步骤2: 接近抓取点')
        if not self.plan_to_pose(grasp_pose, constraints):
            self.get_logger().error('接近抓取点规划失败')
            return False
        
        # 4. 模拟闭合夹爪
        self.get_logger().info('步骤3: 闭合夹爪（模拟）')
        time.sleep(1.0)
        
        # 5. 提升物体
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = "base_link"
        lift_pose.pose.position.x = object_pose.pose.position.x
        lift_pose.pose.position.y = object_pose.pose.position.y
        lift_pose.pose.position.z = object_pose.pose.position.z + self.get_parameter('pre_grasp_height').value
        lift_pose.pose.orientation = pre_grasp_pose.pose.orientation
        
        self.get_logger().info('步骤4: 提升物体')
        if not self.plan_to_pose(lift_pose, constraints):
            self.get_logger().error('提升物体规划失败')
            return False
        
        self.get_logger().info('=== 抓取流程完成 ===')
        return True
    
    def return_home(self):
        """返回初始位置"""
        self.get_logger().info('返回初始位置')
        self.get_logger().info('返回 home (简化实现)')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = GraspController()
        
        time.sleep(2.0)
        
        test_pose = PoseStamped()
        test_pose.header.frame_id = "base_link"
        test_pose.pose.position.x = 0.5
        test_pose.pose.position.y = 0.2
        test_pose.pose.position.z = 0.05
        
        controller.grasp_object(test_pose)
        
    except KeyboardInterrupt:
        print('用户中断')
    except Exception as e:
        print(f'错误: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
