# Vision Grasp Demo - UR5机器人视觉抓取仿真

## 项目概述
基于UR5e机械臂与相机集成，在Gazebo中实现视觉抓取功能仿真。项目通过在官方`ur_simulation_gazebo`基础上二次开发，构建完整的视觉抓取系统。

---

## 开发背景与解决方案

### 初始问题
机械臂各刚体、关节位置错乱，整体上下颠倒，底部无法正常接触地面。

### 问题根源
配置关系理解不足，需深入掌握`ur_simulation_gazebo`项目结构与参数传递机制。

### 解决方案
基于官方项目进行二次开发，精简冗余结构与参数，仅保留核心功能（Gazebo仿真、机械臂类型设置等），利用底层`ur_macro`的默认值简化配置。

---

## 系统架构与开发进度

### 总体流程
1. **环境感知** → 相机获取环境图像
2. **目标识别** → 图像处理识别抓取目标
3. **路径规划** → 计算机械臂运动轨迹
4. **执行抓取** → 控制机械臂完成抓取
5. **反馈调整** → 根据传感器数据优化策略

### 当前进度
- [x] UR5e基础仿真环境搭建
- [x] 启动文件参数优化
- [x] URDF模型文件精简
- [x] 相机模型集成
- [x] 图像处理与目标识别算法
- [x] 位姿估计 + 手眼标定简化版
- [ ] 抓取路径规划算法
- [ ] 完整闭环测试

---

## 核心技术实现

### XACRO参数处理机制
**特性**：顶层`<robot>`标签不应用同文件中的参数默认值，这是XACRO工具本身的特性（官方UR描述文件同样存在此行为）。

**解决方法**：即使定义了`<xacro:arg name="name" default="ur"/>`，运行时仍需显式传递参数。

**验证命令**：
```bash
ros2 run xacro xacro install/vision_grasp_demo/share/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro name:=ur5e
```

---

### 摄像头配置
- **安装位置**：`tool0`坐标系（ROS-I标准工具坐标系）
- **相对位置**：工具中心点前方3cm (XYZ: 0.0, 0.0, 0.03)
- **话题名称**：图像`/image_raw`，相机信息`/camera_info`
- **参数**：分辨率800x600，帧率30Hz，水平视场角约80°

**安装标准**：机械臂整体位于YZ平面，机械爪tool0朝Y轴，运动主要在XZ平面，X的方向，摄像头需朝向X轴（调整关节）并适当突出以避免遮挡（调整位置）。

---

### Gazebo材质显示
**问题**：URDF中定义的`<material>`标签在Gazebo中无法正确显示颜色。

**解决方案**：在`<gazebo>`标签中使用Gazebo专用材质定义：
```xml
<gazebo reference="camera_link">
  <material>Gazebo/Red</material>
</gazebo>
```
**可用材质**：`Gazebo/Red`、`Gazebo/Blue`、`Gazebo/Green`等预定义材质。

---

### 坐标系定义

#### ROS标准坐标系
- **camera_link**：X轴朝前（决定图像主方向），Y轴朝左，Z轴朝上
- **camera_link_optical**：Z轴朝前（光轴），X轴朝右，Y轴朝下，通过固定变换用于标准算法

#### UR机器人tool0坐标系
- Z轴朝前（工具法兰延伸方向）
- X/Y轴按右手定则确定（通常X向右，Y向下）

---

### RViz配置
项目包含自定义RViz配置文件`rviz/ur5_with_camera.rviz`，启动时自动加载：
- RobotModel显示机器人模型
- TF显示坐标变换关系
- Image显示摄像头图像（启用时）

启动文件已修改为使用项目内配置，而非官方`ur_description`包中的配置。

---

### 机器人初始位置配置

**配置文件**：`config/initial_positions.yaml`

**优化配置**：
```yaml
shoulder_pan_joint: 0.0      # 基座旋转
shoulder_lift_joint: -1.57   # 肩部抬升
elbow_joint: 0.0             # 肘部关节
wrist_1_joint: -1.57         # 腕部第一关节
wrist_2_joint: 0.0           # 腕部第二关节
wrist_3_joint: 0.7854        # 腕部第三关节（转45度）
```

**调整目的**：
1. 改善视野：末端向下倾斜45度，便于观察工作台面
2. 优化抓取：确保物体以正确角度进入摄像头视野
3. 扩大工作范围：在机械臂长度限制内最大化有效区域

---

## 编译与运行

### 编译项目
```bash
cd ~/vision_ws
colcon build --packages-select vision_grasp_demo --symlink-install
source install/setup.bash
```

### 启动仿真
```bash
ros2 launch vision_grasp_demo ur5_camera_bringup.launch.py
```

### 验证模型
```bash
ros2 run xacro xacro install/vision_grasp_demo/share/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro name:=ur5e
```

---

## 调试与验证

### 基础调试
```bash
# 启动识别节点
ros2 run vision_grasp_demo object_detector.py

# 查看调试图像
ros2 run rqt_image_view rqt_image_view /debug/detection

# 动态调参
ros2 run rqt_reconfigure rqt_reconfigure

# 录制rosbag用于离线调试
ros2 bag record /camera/image_raw -o detection_test
```

### 带参数文件运行
```bash
ros2 run vision_grasp_demo object_detector.py --ros-args --params-file config/detection_params.yaml
```

### 手动添加物体
```bash
# 从Gazebo "Insert" 菜单选择模型
# 或命令行：
ros2 run gazebo_ros spawn_entity.py \
  -file src/vision_grasp_demo/models/red_box/model.sdf \
  -entity red_box_manual \
  -x 0.3 -y 0.1 -z 0.05
```

---

## 常见问题排查

### 问题1：Gazebo模型姿态未更新
**现象**：修改初始位置配置后，Gazebo中机器人姿态无变化。

**原因**：URDF文件仍引用官方初始位置配置，或未安装`config`文件夹。

**解决方案**：
1. 修改URDF默认路径：
   ```xml
   <xacro:arg name="initial_positions_file" default="$(find vision_grasp_demo)/config/initial_positions.yaml"/>
   ```
2. 确保CMakeLists.txt中安装`config`文件夹
3. 重新编译并重启仿真

**排查经验**：列出所有可能错误原因逐一排查，而非纠结于单一假设。

### 问题2：红色立方体检测不稳定
**现象**：红色立方体上半部分稳定，下半部分因光照阴影抖动断裂。

**原因**：阴影区红色变暗但色相(H)基本不变，饱和度(S)和亮度(V)下限过严。

**解决**：放宽S和V的下限阈值。

---

## 位姿估计流程与坐标系分析
完整手眼标定需9点法，项目周期不允许。采用"预设高度+静态TF"的轻量化方案：
### 总体流程
1. **物体识别**：`object_detector.py`发布像素坐标到`/detection/object_pose_pixel`
2. **位姿估计**：`pose_estimator.py`订阅像素坐标，通过`CameraInfo`获取内参，用光轴距离Zc反投影到相机坐标系
3. **TF变换**：查询`camera_link_optical`到`base_link`的变换，将相机坐标转为基座坐标
4. **发布结果**：发布`/detection/object_pose_3d`，供MoveIt!抓取使用

### 坐标系配置
- **red_box位置**：启动文件中设置为X=0.8, Y=0.3, Z=0.05
- **相机位置**：通过TF查询相机在base_link下的位姿
  ```bash
  ros2 run tf2_ros tf2_echo base_link camera_link_optical
  ```
  
  示例输出：
  ```
  At time 1101.270000000
  - Translation: [0.001, 0.263, 1.079]
  - Rotation: in Quaternion (xyzw) [0.654, -0.653, 0.270, -0.270]
  - Rotation: in RPY (radian) [-2.358, -0.000, -1.571]
  - Rotation: in RPY (degree) [-135.092, -0.000, -89.995]
  ```

### 相机姿态分析
注意我们的相机是倾斜45度的，这影响了深度计算：

**深度计算公式**：
```
Zc = (P_box - P_camera) • Z_axis_camera，结果为1.293
```
其中：
- • = 向量点积
- Z_axis_camera = 相机Z轴在base_link下的单位向量

## 相关资源
- [object_detector.py代码讲解](https://www.kimi.com/share/19af9378-ee82-82e4-8000-0000ef68aae6)
- [Python包安装说明](https://www.kimi.com/share/19af9183-a0a2-8f96-8000-00005849610e)
- [四元数和万向死锁](https://chat.deepseek.com/share/13faqgv85djp8nbebu)
- [成像模型与坐标变换](点击链接查看和 Kimi 的对话 https://www.kimi.com/share/19b22fde-44b2-8ce4-8000-00006ee1502d)