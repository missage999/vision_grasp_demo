from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("vision_grasp_demo"),
                "launch",
                "ur5_camera_bringup.launch.py"
            ])
        ])
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("vision_grasp_demo"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ])
    )

    delay_process = ExecuteProcess(
        cmd=["sleep", "10"],
        name="delay_10s",
    )

    main_grasp_demo_node = Node(
        package="vision_grasp_demo",
        executable="main_grasp_demo.py",
        name="main_grasp_demo",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("vision_grasp_demo"), "config", "grasp_params.yaml"
            ])
        ]
    )

    delay_main_grasp = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=delay_process,
            on_exit=[main_grasp_demo_node],
        ),
    )

    return LaunchDescription([
        bringup_launch,
        moveit_launch,
        delay_process,
        delay_main_grasp,
    ])
