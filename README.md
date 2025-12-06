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
本项目实现基于视觉的机械臂抓取功能，集成UR5e机械臂与相机，在Gazebo中进行仿真测试。整体流程如下：

1. **环境感知**：通过相机获取环境图像信息
2. **目标识别**：处理图像识别抓取目标物体
3. **路径规划**：计算机械臂运动路径
4. **执行抓取**：控制机械臂执行抓取动作
5. **反馈调整**：根据传感器反馈调整抓取策略

### 当前开发进度
- [x] UR5e基础仿真环境搭建
- [x] 启动文件参数优化
- [x] URDF模型文件精简
- [x] 相机模型集成
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

这就是为什么即使我们定义了`<xacro:arg name="name" default="ur"/>`，在运行命令时仍然需要传递name:=ur5e参数的原因。这不是我们的代码有问题，而是XACRO工具的工作方式决定的。

### 正确的验证命令

应该写成：ros2 run xacro xacro install/vision_grasp_demo/share/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro name:=ur5e

### 启动文件验证

不过启动的时候：
- 验证ur5e基础仿真：`ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e`
- 验证ur5e基础仿真：`ros2 launch ur_simulation_gazebo ur_sim_control.launch.py`都可以

### 摄像头配置说明

摄像头已成功集成到UR5机器人模型中，配置详情如下：

1. **安装位置**：摄像头安装在`tool0`坐标系上，这是ROS-I标准的工具坐标系
2. **相对位置**：位于工具中心点前方1.5厘米处（XYZ: 0.0, 0.0, 0.015）
3. **话题名称**：
   - 图像话题：`/image_raw`
   - 摄像头信息话题：`/camera_info`
4. **参数配置**：
   - 分辨率：800x600
   - 帧率：30Hz
   - 水平视场角：约80度
5. **畸变参数**：已移除默认为0的畸变参数，保持配置简洁

### Gazebo显示问题说明

在Gazebo中，URDF模型的材质显示需要特殊处理：

1. **问题描述**：在URDF中定义的`<material>`标签在Gazebo中可能无法正确显示颜色
2. **解决方案**：需要在`<gazebo>`标签中使用Gazebo专用的材质定义
3. **实现方式**：
   ```xml
   <!-- Gazebo material for camera link -->
   <gazebo reference="camera_link">
     <material>Gazebo/Red</material>
   </gazebo>
   ```
4. **可用材质**：Gazebo提供了多种预定义材质，如`Gazebo/Red`、`Gazebo/Blue`、`Gazebo/Green`等

### 坐标系说明

在ROS和机器人系统中，坐标系的定义对于正确理解传感器数据和机器人的空间关系至关重要。

#### ROS标准坐标系定义

**camera_link坐标系**：
- X轴：朝前（决定图像的主要方向）
- Y轴：朝左
- Z轴：朝上
- 图像方向由camera_link的X轴方向决定，只需将X轴对准需要的方向即可

**camera_link_optical坐标系**：
- Z轴：朝前（光轴方向）
- X轴：朝右
- Y轴：朝下
- 相对于camera_link的变换是固定的，主要用于标准算法使用
- 通过`<frameName>camera_link_optical</frameName>`指定，不影响图像显示方向

#### UR机器人tool0标准坐标系

**tool0坐标系**：
- Z轴：朝前（工具法兰的延伸方向）
- X轴和Y轴：按右手定则确定（通常X轴向右，Y轴向下）

摄像头安装在tool0坐标系上，利用其作为末端执行器的标准参考点，有利于传感器数据对齐和后续视觉伺服控制。

### RViz配置说明

项目现在包含自定义的RViz配置文件，位于`rviz/ur5_with_camera.rviz`。该配置文件已在启动文件中正确引用，启动时会自动加载以下配置：

1. **显示设置**：
   - RobotModel显示机器人模型
   - TF显示坐标变换关系
   - Image显示摄像头图像（如果启用了摄像头）

2. **摄像头图像面板**：
   - 自动订阅`/image_raw`话题
   - 设置合适的图像显示尺寸

3. **视图设置**：
   - 默认视角朝向机器人工作区域
   - 适当缩放比例便于观察

启动文件已修改为使用项目内的RViz配置文件，而非官方`ur_description`包中的配置文件。

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

# 启动UR5e与相机仿真环境
ros2 launch vision_grasp_demo ur5_camera_bringup.launch.py
```

### 验证模型文件
```bash
# 设置环境变量
source install/setup.bash

# 验证XACRO文件语法
ros2 run xacro xacro install/vision_grasp_demo/share/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro name:=ur5e
```

### 查看摄像头图像（在RViz中）
启动仿真环境后，可以在RViz中添加Image面板，订阅`/image_raw`话题查看摄像头实时图像。