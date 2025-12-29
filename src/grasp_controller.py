#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit.planning import MoveGroupInterface
from moveit_msgs.msg import CollisionObject, PlanningScene, Constraints, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import time

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')
        
        # 参数
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('pre_grasp_height', 0.2)  # 预抓取高度（米）
        self.declare_parameter('grasp_height', 0.1)      # 抓取高度
        
        # MoveIt接口
        group_name = self.get_parameter('planning_group').value
        self.move_group = MoveGroupInterface(
            group_name,
            'base_link',
            joint_states_topic='/joint_states',
            planning_pipelines=['ompl'],
        )
        
        # PlanningScene接口
        self.planning_scene_interface = self.move_group.get_planning_scene_interface()
        
        # 发布规划场景
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        self.get_logger().info(f'GraspController已启动 - 规划组: {group_name}')
        
        # 等待MoveIt服务就绪
        self.move_group.wait_for_planning_scene()
        self.add_collision_objects()
    
    def add_collision_objects(self):
        """添加碰撞体：地面"""
        self.get_logger().info('添加碰撞场景...')
        
        # 地面
        ground = CollisionObject()
        ground.id = "ground"
        ground.header.frame_id = "base_link"
        ground.operation = CollisionObject.ADD
        
        ground_pose = Pose()
        ground_pose.position.z = -0.01
        
        ground.primitive_poses = [ground_pose]
        ground.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0, 3.0, 0.01])]
        
        self.planning_scene_interface.apply_collision_object(ground)
        
        time.sleep(1.0)  # 等待场景更新
    
    def plan_to_pose(self, target_pose, constraints=None):
        """通用规划函数"""
        self.move_group.set_pose_target(target_pose)
        
        if constraints:
            self.move_group.set_path_constraints(constraints)
        
        # 规划参数
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(3)
        self.move_group.set_goal_tolerance(0.01)  # 1cm误差容忍
        
        # 执行规划和执行
        self.get_logger().info('规划中...')
        plan_result = self.move_group.plan()
        
        if plan_result.planning_result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('规划成功，执行中...')
            self.move_group.execute(plan_result.trajectory, wait=True)
            return True
        else:
            self.get_logger().error(f'规划失败: {plan_result.planning_result.error_code.val}')
            return False
    
    def grasp_object(self, object_pose: PoseStamped):
        """主抓取流程"""
        self.get_logger().info('=== 开始抓取流程 ===')
        
        # 1. 移动到预抓取位置（物体上方）
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = "base_link"
        pre_grasp_pose.pose.position = object_pose.pose.position
        pre_grasp_pose.pose.position.z += self.get_parameter('pre_grasp_height').value
        
        # 保持垂直向下的朝向（简化）
        pre_grasp_pose.pose.orientation.x = 0.0
        pre_grasp_pose.pose.orientation.y = 0.707  # 90度旋转
        pre_grasp_pose.pose.orientation.z = 0.0
        pre_grasp_pose.pose.orientation.w = 0.707
        
        self.get_logger().info('步骤1: 移动到预抓取点')
        if not self.plan_to_pose(pre_grasp_pose):
            return False
        
        # 2. 设置方向约束（保持末端垂直）
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "tool0"
        orientation_constraint.orientation = pre_grasp_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 3.14  # 允许绕Z轴旋转
        orientation_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.orientation_constraints = [orientation_constraint]
        
        # 3. 接近物体
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.pose = object_pose.pose
        grasp_pose.pose.position.z += self.get_parameter('grasp_height').value
        
        self.get_logger().info('步骤2: 接近抓取点')
        if not self.plan_to_pose(grasp_pose, constraints):
            return False
        
        # 4. 模拟闭合夹爪（UR5无夹爪，用等待代替）
        self.get_logger().info('步骤3: 闭合夹爪（模拟）')
        time.sleep(2.0)
        
        # 5. 提升物体
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = "base_link"
        lift_pose.pose = object_pose.pose
        lift_pose.pose.position.z += self.get_parameter('pre_grasp_height').value  # 提升预抓取高度
        
        self.get_logger().info('步骤4: 提升物体')
        if not self.plan_to_pose(lift_pose, constraints):
            return False
        
        self.get_logger().info('=== 抓取流程完成 ===')
        return True
    
    def return_home(self):
        """返回初始位置"""
        home_pose = PoseStamped()
        home_pose.header.frame_id = "base_link"
        
        # 使用已命名的"home"位姿
        self.move_group.set_named_target("home")
        plan_result = self.move_group.plan()
        
        if plan_result.planning_result.error_code.val == 1:
            self.move_group.execute(plan_result.trajectory, wait=True)


def main(args=None):
    rclpy.init(args=args)
    controller = GraspController()
    
    # 测试用：等待2秒后自动执行
    time.sleep(2.0)
    controller.grasp_object(PoseStamped())  # 需传入真实位姿
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()