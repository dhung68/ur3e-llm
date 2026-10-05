from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Đường dẫn tới world của bài 2.
    default_world = PathJoinSubstitution([
        FindPackageShare("hri_bai2_environment"),
        "worlds",
        "bai2.sdf",
    ])

    ur_type = LaunchConfiguration("ur_type")

    # Mở Gazebo, tạo robot và khởi động controller.
    gazebo_and_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "launch",
                "ur_sim_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "world_file": LaunchConfiguration("world_file"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "launch_rviz": "false",
            "description_package": "hri_ur3e_description",
            "description_file": "ur3e_gripper.urdf.xacro",
            "runtime_config_package": "hri_ur3e_description",
            "controllers_file": "ur_gripper_controllers.yaml",
        }.items(),
    )

    # Mở MoveIt và RViz.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "description_package": "hri_ur3e_description",
            "description_file": "ur3e_gripper.urdf.xacro",
        }.items(),
    )

    # Tự thêm bàn và cube vào MoveIt.
    initialize_scene = Node(
        package="hri_bai2_environment",
        executable="initialize_scene.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # YAML declares the controller type; a spawner must also activate it.
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "10",
            "--switch-timeout", "10",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=default_world,
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        gazebo_and_control,
        gripper_spawner,
        moveit,
        initialize_scene,
    ])
