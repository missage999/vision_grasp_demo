from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_name = LaunchConfiguration("robot_name")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # 固定控制器配置路径
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gazebo"), "config", "ur_controllers.yaml"]
    )

    # 固定初始位置文件路径
    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare("vision_grasp_demo"), "config", "initial_positions.yaml"]
    )

    # 固定RViz配置文件路径
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("vision_grasp_demo"), "rviz", "ur5_with_camera.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "name:=",
            robot_name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "ur", "-topic", "robot_description"],
        output="screen",
    )

    # Spawn red box model
    gazebo_spawn_red_box = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_red_box",
        arguments=[
            "-entity", "red_box",
            "-file", PathJoinSubstitution([
                FindPackageShare("vision_grasp_demo"), "models", "red_box", "model.sdf"
            ]),
            "-x", "0.8",
            "-y", "0.3",
            "-z", "0.05"  # 放置在地面上方一点，避免与地面碰撞
        ],
        output="screen",
    )

    # Object detection node
    object_detector_node = Node(
        package="vision_grasp_demo",
        executable="object_detector.py",
        name="object_detector",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("vision_grasp_demo"), "config", "detection_params.yaml"
            ])
        ]
    )

    # RQT Image View node for debugging
    rqt_image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=["/debug/detection"],
        output="screen"
    )

    # Pose estimation node
    pose_estimator_node = Node(
        package='vision_grasp_demo',
        executable='pose_estimator.py',
        name='pose_estimator',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('vision_grasp_demo'), 'config', 'pose_estimation.yaml'
        ])]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        #delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gazebo,
        gazebo_spawn_robot,
        gazebo_spawn_red_box,  # 添加红盒子生成节点
        object_detector_node,   # 添加物体检测节点
        rqt_image_view_node,    # 添加图像查看节点
        # Add pose estimator node
        pose_estimator_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur",
            description="Name of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur12e",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="vision_grasp_demo",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur5_with_camera.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])