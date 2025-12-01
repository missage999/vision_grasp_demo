验证ur5基础仿真：ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5

验证文件语法：ros2 run xacro xacro ~/vision_ws/src/vision_grasp_demo/urdf/ur5_with_camera.urdf.xacro

问题：机械臂各刚体、关节位置错乱，而且整体上下颠倒。正常底部接触地面，底部是边缘外部有一根小圆柱。
原因：可能是配置关系，先理解官方项目ur_simulation_gazebo的所有内容，然后查看moviit2_tutorials的教程，最后再思考自己的项目如何实现。