from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包目录
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('vision_grasp_demo')

    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5_with_camera.urdf.xacro')

    # 参数声明
    camera_name = LaunchConfiguration('camera_name')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    # 参数声明
    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name'
    )

    declare_camera_width_cmd = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera width'
    )

    declare_camera_height_cmd = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera height'
    )

    declare_camera_fps_cmd = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera fps'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world'),
        description='SDF world file'
    )

    # Robot description
    robot_description = Command([
        'xacro ', urdf_file,
        ' camera_name:=', camera_name,
        ' camera_width:=', camera_width,
        ' camera_height:=', camera_height,
        ' camera_fps:=', camera_fps
    ])

    return LaunchDescription([
        declare_camera_name_cmd,
        declare_camera_width_cmd,
        declare_camera_height_cmd,
        declare_camera_fps_cmd,
        declare_use_rviz_cmd,
        declare_world_cmd,

        # 启动Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        # 机器人状态发布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}]
        ),

        # Spawn实体
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'ur5_with_camera',
                       '-topic', 'robot_description',
                       '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),

        # RViz（可选）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz)
        )
    ])
