"""MoveIt overlay for the existing UR arm and physical two-finger gripper."""

from copy import deepcopy
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    original = Path(get_package_share_directory("ur_moveit_config"))
    description = Path(get_package_share_directory("hri_ur3e_description"))
    own = Path(get_package_share_directory("hri_bai2_environment"))

    def load(path):
        return yaml.safe_load(path.read_text())

    robot = xacro.process_file(
        str(description / "urdf/ur3e_gripper.urdf.xacro"),
        mappings={"name": "ur", "ur_type": ur_type, "safety_limits": "true"},
    ).toxml()
    semantic = xacro.process_file(
        str(own / "srdf/ur3e_gripper.srdf.xacro"), mappings={"name": "ur"}
    ).toxml()
    overlay = load(own / "config/moveit_gripper.yaml")
    limits = load(original / "config/joint_limits.yaml")
    limits["joint_limits"].update(overlay["joint_limits"])
    kinematics = load(original / "config/kinematics.yaml")["/**"]["ros__parameters"]
    kinematics["robot_description_kinematics"]["ur_grasp"] = deepcopy(
        kinematics["robot_description_kinematics"]["ur_manipulator"]
    )
    # A slightly longer search budget for the fixed offset at grasp_tcp.
    kinematics["robot_description_kinematics"]["ur_grasp"]["kinematics_solver_timeout"] = 0.1
    ompl = load(original / "config/ompl_planning.yaml")
    ompl["ur_grasp"] = deepcopy(ompl["ur_manipulator"])
    ompl["gripper"] = {"planner_configs": ["RRTConnectkConfigDefault"]}
    pipeline = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
        **ompl,
    }}
    controllers = load(original / "config/controllers.yaml")
    controllers["controller_names"] = ["joint_trajectory_controller", "gripper_controller"]
    controllers["joint_trajectory_controller"]["default"] = True
    controllers["scaled_joint_trajectory_controller"]["default"] = False
    controllers["gripper_controller"] = overlay["controller"]
    common = [
        {"robot_description": robot, "robot_description_semantic": semantic,
         "publish_robot_description_semantic": True, "use_sim_time": True},
        kinematics, {"robot_description_planning": limits}, pipeline,
    ]
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=common + [
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
             "moveit_simple_controller_manager": controllers,
             "moveit_manage_controllers": False,
             "trajectory_execution.allowed_execution_duration_scaling": 1.2,
             "trajectory_execution.allowed_goal_duration_margin": 0.5,
             "trajectory_execution.allowed_start_tolerance": 0.01,
             "trajectory_execution.execution_duration_monitoring": False,
             "publish_planning_scene": True, "publish_geometry_updates": True,
             "publish_state_updates": True, "publish_transforms_updates": True},
        ],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
        condition=IfCondition(LaunchConfiguration("moveit_launch_rviz")),
        arguments=["-d", str(own / "config/bai2_view.rviz")], parameters=common,
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("moveit_launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
