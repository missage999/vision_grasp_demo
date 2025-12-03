# Vision Grasp Demo - UR5机器人视觉抓取仿真

## 项目背景与问题分析

### 初始问题
问题：机械臂各刚体、关节位置错乱，而且整体上下颠倒。正常底部接触地面，底部是边缘外部有一根小圆柱。

### 问题原因分析
原因：可能是配置关系，先理解官方项目ur_simulation_gazebo的所有内容，然后查看moviit2_tutorials的教程，最后再思考自己的项目如何实现。

## 解决方案与实施过程

解决：理解了ur_simulation_gazebo项目，直接在官方项目的基础上二次开发。官方项目较大，彻底理解项目结构和运行流程，特别是参数传递，从而删除不必要的结构和参数，只保留我们需要的。这个过程的原则是，基于gazebo项目、机械臂的类型以及一些简单参数保留和设置，其余参数因为和底层ur_macro有默认值又无需更改而删除。

至此，基础结构就梳理完成，可以开始添加摄像头camera。

## 视觉抓取项目总体流程与进度

### 项目总体架构
本项目实现基于视觉的机械臂抓取功能，集成UR5机械臂与相机，在Gazebo中进行仿真测试。整体流程如下：

1. **环境感知**：通过相机获取环境图像信息
2. **目标识别**：处理图像识别抓取目标物体
3. **路径规划**：计算机械臂运动路径
4. **执行抓取**：控制机械臂执行抓取动作
5. **反馈调整**：根据传感器反馈调整抓取策略

### 当前开发进度
- [x] UR5基础仿真环境搭建
- [x] 启动文件参数优化
- [x] URDF模型文件精简
- [ ] 相机模型集成
- [ ] 图像处理与目标识别算法
- [ ] 抓取路径规划算法
- [ ] 完整闭环测试

## 技术细节与经验总结

### XACRO参数处理机制

验证文件语法：ros2 run xacro xacro ~/vision_ws/src/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro

问题：缺少name

XACRO处理机制：XACRO在处理顶层<robot>标签时，不会应用在同一文件中定义的参数默认值

普遍现象：这不是我们代码的问题，而是XACRO工具本身的特性，连官方的UR描述文件也有同样的行为

解决方法：虽然定义默认值是一种良好的实践，但在实际使用时仍需要显式传递参数

这就是为什么即使我们定义了`<xacro:arg name="name" default="ur"/>`，在运行命令时仍然需要传递name:=ur5参数的原因。这不是我们的代码有问题，而是XACRO工具的工作方式决定的。

### 正确的验证命令

应该写成：ros2 run xacro xacro install/vision_grasp_demo/share/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro name:=ur5

### 启动文件验证

不过启动的时候：
- 验证ur5基础仿真：`ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5`
- 验证ur5基础仿真：`ros2 launch ur_simulation_gazebo ur_sim_control.launch.py`都可以

## 编译运行步骤

### 编译项目
```bash
cd ~/vision_ws
colcon build --packages-select vision_grasp_demo
```

### 运行仿真环境
```bash
# 设置环境变量
source install/setup.bash

# 启动UR5与相机仿真环境
ros2 launch vision_grasp_demo ur5_camera_bringup.launch.py
```

### 验证模型文件
```bash
# 设置环境变量
source install/setup.bash

# 验证XACRO文件语法
ros2 run xacro xacro install/vision_grasp_demo/share/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro name:=ur5
```