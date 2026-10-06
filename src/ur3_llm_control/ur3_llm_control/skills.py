"""Low-speed MoveIt skills, with physical grasp verification in Gazebo."""
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, Constraints,
    JointConstraint, PlanningScene, PlanningSceneComponents, RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .state import GazeboStateProvider

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
FINGERS = ["left_finger_joint", "right_finger_joint"]
TOUCH_LINKS = ["left_finger", "right_finger"]
# IK seed observed in successful red_cube -> zone_a trials. This selects a
# tested elbow branch; only collision-checked IK and OMPL results are executed.
GRASP_IK_SEED = [3.1, -1.324, -2.177, -1.212, 1.571, -1.611]


def pose_data(pose):
    return {"position": [pose.position.x, pose.position.y, pose.position.z],
            "quaternion": [pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w]}


def matrix(pose):
    x, y, z, w = pose_data(pose)["quaternion"]
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = pose_data(pose)["position"]
    return result


def pose_from_matrix(value):
    # scipy is unnecessary; use a stable matrix-to-quaternion conversion.
    from scipy.spatial.transform import Rotation
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, value[:3, 3])
    q = Rotation.from_matrix(value[:3, :3]).as_quat()
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, q)
    return result


@dataclass
class SkillResult:
    success: bool
    skill: str
    reason: str
    held_object: str | None
    holding_verified: bool
    pending_object: str | None
    recovery_required: bool


class Skills(Node):
    def __init__(self, evidence_dir, state_provider=None, screenshots=False,
                 compact_evidence=False, node_name="ur3e_skills",
                 scene_config=None, provider_factory=None):
        super().__init__(node_name, parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.compact_evidence = compact_evidence
        self.evidence = Path(evidence_dir)
        self.evidence.mkdir(parents=True, exist_ok=True)
        self.events = []
        self.results = []
        self.held_object = None
        self.holding_verified = False
        self.relative_object = None
        self.contact_object = None
        self.original_acm = None
        self.last_joint_wall = 0.0
        self.joints = {}
        self.velocities = {}
        self.screenshots = screenshots
        self.config = deepcopy(scene_config) if scene_config is not None else json.loads((Path(get_package_share_directory("hri_bai2_environment")) /
                                  "config/scene.json").read_text())
        self.provider = state_provider or (provider_factory(self) if provider_factory else GazeboStateProvider(self))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subscription = self.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory,
                                    "/gripper_controller/follow_joint_trajectory")
        self.wait_ready()
        self.verify_geometry()
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        attachments = self.service(GetPlanningScene, "/get_planning_scene", req).scene.robot_state.attached_collision_objects
        if attachments:
            self.contact_object = attachments[0].object.id
            self.record("existing_attachment_unverified", objects=[a.object.id for a in attachments])

    def _joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.velocities = dict(zip(msg.name, msg.velocity))
        self.last_joint_wall = time.monotonic()

    def wait(self, future, timeout=15.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"ROS request exceeded {timeout}s")
        return future.result()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def service(self, typ, name, request, timeout=15.0):
        client = self.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Missing service {name}")
            return self.wait(client.call_async(request), timeout)
        finally:
            self.destroy_client(client)

    def wait_ready(self):
        for client in (self.move, self.execute, self.gripper):
            if not client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError(f"Missing action {client._action_name}")
        deadline = time.monotonic() + 15.0
        last_error = "Waiting for joint states"
        while time.monotonic() < deadline:
            self.spin(0.1)
            try:
                self.provider.get("red_cube")
                self.tcp_pose()
                if all(n in self.joints for n in ARM + FINGERS):
                    for name in FINGERS:
                        if not -1e-6 <= self.joints[name] <= 0.020001:
                            raise RuntimeError(f"Finger state outside physical limits: {name}={self.joints[name]}")
                    return
            except Exception as error:
                last_error = str(error)
        raise RuntimeError(f"No fresh safe cube/joint/TF state after 15s: {last_error}")

    def verify_geometry(self):
        path = Path(get_package_share_directory("hri_ur3e_description")) / "urdf/simple_gripper.xacro"
        root = ET.parse(path).getroot()
        ns = "{http://www.ros.org/wiki/xacro}"
        gripper = root.find(f"{ns}macro[@name='hri_simple_gripper']")
        finger = gripper.find(f"{ns}hri_box_link[@name='${{prefix}}left_finger']")
        origin = gripper.find("joint[@name='${prefix}left_finger_joint']/origin")
        tcp = gripper.find("joint[@name='${prefix}grasp_tcp_joint']/origin")
        self.tip_extra = float(origin.get("xyz").split()[2]) + float(finger.get("sz")) - float(tcp.get("xyz").split()[2])
        self.tcp_offset_above_cube = max(0.006, self.tip_extra + 0.004 - 0.015)
        self.record("geometry", finger_beyond_tcp_m=self.tip_extra,
                    grasp_tcp_above_cube_center_m=self.tcp_offset_above_cube,
                    table_clearance_m=0.015 + self.tcp_offset_above_cube - self.tip_extra)

    def tcp_pose(self):
        transform = self.tf.lookup_transform("world", "grasp_tcp", Time())
        pose = Pose()
        p = transform.transform.translation
        pose.position.x, pose.position.y, pose.position.z = p.x, p.y, p.z
        pose.orientation = deepcopy(transform.transform.rotation)
        return pose

    def state(self):
        if time.monotonic() - self.last_joint_wall > 1.0:
            raise RuntimeError("Stale joint states")
        state = RobotState()
        state.joint_state.name = list(self.joints)
        state.joint_state.position = list(self.joints.values())
        state.is_diff = True
        return state

    def record(self, stage, **extra):
        data = {"stage": stage, "sim_time": self.get_clock().now().nanoseconds / 1e9,
                "wall_time": time.time(), "held_object": self.held_object,
                "holding_verified": self.holding_verified, **deepcopy(extra)}
        try:
            data["tcp"] = pose_data(self.tcp_pose())
        except Exception:
            pass
        data["cubes"] = {}
        for name in self.config["cubes"]:
            try:
                data["cubes"][name] = pose_data(self.provider.get(name).pose)
            except Exception as error:
                data["cubes"][name] = {"error": str(error)}
        data["joints"] = dict(self.joints)
        if getattr(self, "compact_evidence", False):
            samples = data.pop("samples", [])
            if samples:
                data["sample_count"] = len(samples)
                for key in ("height_gain_m", "relative_error_m", "held_relative_error_m"):
                    values = [s[key] for s in samples if key in s]
                    if values:
                        data[key + "_min"] = min(values)
                        data[key + "_max"] = max(values)
            self.events.append(data)
            return data
        self.events.append(data)
        (self.evidence / "events.json").write_text(json.dumps(self.events, indent=2))
        brief = {key: value for key, value in extra.items() if key != "samples"}
        if "samples" in extra:
            brief["sample_count"] = len(extra["samples"])
        self.get_logger().info(f"{stage}: {json.dumps(brief)}")
        return data

    def snapshot(self, stage):
        self.spin(0.15)
        self.record(stage)
        if not self.screenshots:
            return
        try:
            from .screenshots import aim_gazebo, capture_windows
            request = None
            try:
                request = aim_gazebo(pose_data(self.tcp_pose())["position"],
                                     pose_data(self.provider.get(getattr(self, "active_object", "red_cube")).pose)["position"])
            except Exception as error:
                # Camera placement is optional. Still capture the actual current
                # GUI view when its camera service is delayed or unavailable.
                self.record("gui_camera_unavailable", for_stage=stage, reason=str(error))
            self.spin(0.1)
            saved = capture_windows(self.evidence, f"{len(self.events):03d}_{stage}")
            self.spin(0.15)  # X11 capture blocks callbacks; refresh measured state afterwards.
            self.record("screenshots", for_stage=stage, files=saved, gui_camera_request=request)
        except Exception as error:
            self.spin(0.2)  # failed GUI calls also blocked ROS callbacks
            self.record("screenshot_unavailable", for_stage=stage, reason=str(error))

    def action(self, client, goal, label, timeout=90.0, require_success=True):
        handle = self.wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            self.record("action", label=label, accepted=False)
            raise RuntimeError(f"{label}: goal rejected")
        future = handle.get_result_async()
        motion_samples = []
        deadline = time.monotonic() + timeout
        last_sample = 0.0
        try:
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{label}: action exceeded {timeout}s")
                if time.monotonic() - last_sample >= 0.1:
                    last_sample = time.monotonic()
                    observed_name = self.held_object or getattr(self, "active_object", "red_cube")
                    actual = self.provider.get(observed_name).pose
                    sample = {"sim_time": self.get_clock().now().nanoseconds/1e9,
                              "object": observed_name, "cube": pose_data(actual), "tcp": pose_data(self.tcp_pose())}
                    if self.holding_verified and self.held_object:
                        carried = self.provider.get(self.held_object).pose
                        relative = np.linalg.inv(matrix(self.tcp_pose())) @ matrix(carried)
                        drift = float(np.linalg.norm(relative[:3, 3] - self.relative_object[:3, 3]))
                        sample["held_relative_error_m"] = drift
                        if drift > 0.008:
                            self.holding_verified = False
                            raise RuntimeError(f"Physical grasp lost during {label}: drift {drift:.4f}m")
                    motion_samples.append(sample)
            result = future.result()
        except Exception as error:
            cancellation = self.wait(handle.cancel_goal_async(), 10.0)
            self.record("action_cancelled", label=label, reason=str(error), samples=motion_samples,
                        cancellation_return_code=cancellation.return_code)
            # Do not start another movement until cancellation has a terminal result.
            self.wait(future, 10.0)
            raise
        code = result.result.error_code
        code = code.val if hasattr(code, "val") else code
        self.record("action", label=label, accepted=True, status=result.status, error_code=code,
                    samples=motion_samples)
        expected = 0 if isinstance(result.result, FollowJointTrajectory.Result) else 1
        if require_success and (result.status != GoalStatus.STATUS_SUCCEEDED or code != expected):
            raise RuntimeError(f"{label}: status {result.status}, error {code}")
        self.spin(0.3)
        return result

    def set_gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = FINGERS
        point = JointTrajectoryPoint(positions=[position, position], velocities=[0.0, 0.0])
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        return self.action(self.gripper, goal, f"gripper_{position}", 15.0)

    def move_joints(self, targets, label, group="ur_grasp", names=ARM, scaling=0.05):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = scaling
        goal.request.max_acceleration_scaling_factor = scaling
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        constraints.joint_constraints = [JointConstraint(
            joint_name=name, position=float(target), tolerance_above=0.001,
            tolerance_below=0.001, weight=1.0) for name, target in zip(names, targets)]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.action(self.move, goal, label, 120.0)
        errors = {}
        for name, target in zip(names, targets):
            difference = self.joints[name] - target
            errors[name] = abs(math.atan2(math.sin(difference), math.cos(difference))) if name in ARM else abs(difference)
        self.record("joint_target_check", label=label, errors=errors)
        if any(error > (0.005 if name in ARM else 0.002) for name,error in errors.items()):
            raise RuntimeError(f"{label}: measured joints did not reach requested target")

    def move_pose(self, pose, label, ik_seed=None):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_grasp"
        request.ik_request.ik_link_name = "grasp_tcp"
        request.ik_request.pose_stamped = PoseStamped(pose=pose)
        request.ik_request.pose_stamped.header.frame_id = "world"
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        request.ik_request.robot_state = self.state()
        if ik_seed is not None:
            state = request.ik_request.robot_state.joint_state
            for name, value in zip(ARM, ik_seed):
                state.position[state.name.index(name)] = value
            self.record("ik_seed", label=label, joints=dict(zip(ARM, ik_seed)))
        response = self.service(GetPositionIK, "/compute_ik", request)
        self.record("ik", label=label, error_code=response.error_code.val, target=pose_data(pose))
        if response.error_code.val != 1:
            raise RuntimeError(f"{label}: no collision-free TCP IK ({response.error_code.val})")
        positions = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        # KDL may return an equivalent angle one full turn away on bounded UR
        # revolute joints. Select the nearest legal representation before OMPL.
        import xacro
        urdf_path = Path(get_package_share_directory("hri_ur3e_description")) / "urdf/ur3e_gripper.urdf.xacro"
        urdf = ET.fromstring(xacro.process_file(str(urdf_path), mappings={"name": "ur", "ur_type": "ur3e"}).toxml())
        targets = []
        for name in ARM:
            joint = urdf.find(f"joint[@name='{name}']")
            limit = joint.find("limit")
            candidates = [positions[name] + 2 * math.pi * k for k in range(-3, 4)]
            if joint.get("type") != "continuous":
                candidates = [q for q in candidates if float(limit.get("lower")) <= q <= float(limit.get("upper"))]
            if not candidates:
                raise RuntimeError(f"IK has no legal angle representation for {name}")
            targets.append(min(candidates, key=lambda q: abs(q-self.joints[name])))
        self.record("ik_normalized", label=label, original=positions, targets=dict(zip(ARM, targets)))
        self.move_joints(targets, label)
        self.verify_tcp(pose)

    def verify_tcp(self, target, tolerance=0.005):
        actual = self.tcp_pose()
        error = np.linalg.norm(matrix(actual)[:3, 3] - matrix(target)[:3, 3])
        qa = np.array(pose_data(actual)["quaternion"])
        qt = np.array(pose_data(target)["quaternion"])
        angle = 2 * math.acos(float(np.clip(abs(np.dot(qa, qt)), 0.0, 1.0)))
        self.record("tcp_check", position_error_m=float(error), orientation_error_rad=angle)
        if error > tolerance or angle > 0.05:
            raise RuntimeError(f"TCP error {error:.4f}m / {angle:.4f}rad")

    def cartesian(self, target, label):
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.group_name = "ur_grasp"
        request.link_name = "grasp_tcp"
        request.start_state = self.state()
        request.waypoints = [target]
        request.max_step = 0.004
        request.jump_threshold = 2.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.03
        request.max_acceleration_scaling_factor = 0.03
        starting_tcp = self.tcp_pose()
        response = self.service(GetCartesianPath, "/compute_cartesian_path", request, 20.0)
        self.record("cartesian_plan", label=label, fraction=response.fraction,
                    error_code=response.error_code.val,
                    points=len(response.solution.joint_trajectory.points))
        if response.error_code.val != 1 or response.fraction < 0.999:
            raise RuntimeError(f"{label}: incomplete collision-free path {response.fraction}")
        # Preserve MoveIt's velocity/acceleration-consistent timing, slowing it
        # further when necessary to bound Cartesian average speed to 12mm/s.
        trajectory = response.solution.joint_trajectory
        elapsed = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec / 1e9
        distance = float(np.linalg.norm(matrix(target)[:3, 3] - matrix(starting_tcp)[:3, 3]))
        factor = max(1.0, distance / 0.012 / max(elapsed, 1e-6))
        for point in trajectory.points:
            t = (point.time_from_start.sec + point.time_from_start.nanosec / 1e9) * factor
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t % 1) * 1e9)
            point.velocities = [v/factor for v in point.velocities]
            point.accelerations = [a/factor**2 for a in point.accelerations]
        elapsed *= factor
        self.record("cartesian_timing", label=label, duration_sim_s=elapsed, slowdown=factor)
        goal = ExecuteTrajectory.Goal(trajectory=response.solution)
        self.action(self.execute, goal, label, max(45.0, elapsed * 2 + 15))
        self.verify_tcp(target)

    def cube_object(self, name, pose, frame="world"):
        obj = CollisionObject(id=name)
        obj.header.frame_id = frame
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=self.config["cubes"][name]["size"])]
        obj.pose = deepcopy(pose)
        shape_pose = Pose()
        shape_pose.orientation.w = 1.0
        obj.primitive_poses = [shape_pose]
        obj.operation = CollisionObject.ADD
        return obj

    def apply_scene(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        response = self.service(ApplyPlanningScene, "/apply_planning_scene",
                                ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise RuntimeError("MoveIt rejected planning scene update")

    def sync_scene(self):
        scene = PlanningScene()
        for name in self.config["cubes"]:
            if name != self.contact_object:
                scene.world.collision_objects.append(self.cube_object(name, self.provider.get(name).pose))
        self.apply_scene(scene)
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        from rosidl_runtime_py.convert import message_to_ordereddict
        deadline = time.monotonic() + 10.0
        while True:
            observed = self.service(GetPlanningScene, "/get_planning_scene", request).scene
            if not getattr(self, "compact_evidence", False):
                (self.evidence / f"scene_check_{len(self.events):03d}.json").write_text(
                    json.dumps(message_to_ordereddict(observed), indent=2))
            planning_joints = dict(zip(observed.robot_state.joint_state.name, observed.robot_state.joint_state.position))
            joint_error = {}
            for name in ARM + FINGERS:
                if name not in planning_joints:
                    joint_error[name] = float("inf")
                    continue
                delta = planning_joints[name] - self.joints[name]
                joint_error[name] = abs(math.atan2(math.sin(delta), math.cos(delta))) if name in ARM else abs(delta)
            if all(joint_error[name] <= (0.01 if name in ARM else 0.001) for name in ARM + FINGERS):
                break
            self.record("wait_for_moveit_state", errors={n: e if math.isfinite(e) else "missing" for n,e in joint_error.items()})
            if time.monotonic() >= deadline:
                raise RuntimeError("MoveIt / joint_states still disagree after 10s; motion refused")
            # Action availability does not mean MoveIt's state monitor has received
            # its first joint_states yet. Wait for measured state, keeping thresholds.
            self.spin(0.2)
        cube_errors = {}
        for obj in observed.world.collision_objects:
            if obj.id in self.config["cubes"] and obj.id != self.contact_object:
                if obj.header.frame_id != "world" or not obj.primitive_poses:
                    raise RuntimeError(f"Unexpected planning frame/geometry for {obj.id}")
                cube_errors[obj.id] = float(np.linalg.norm((matrix(obj.pose) @ matrix(obj.primitive_poses[0]))[:3, 3] -
                                                          matrix(self.provider.get(obj.id).pose)[:3, 3]))
                if cube_errors[obj.id] > 0.001:
                    raise RuntimeError(f"Gazebo / planning scene disagree for {obj.id}")
        expected = set(self.config["cubes"]) - {self.contact_object}
        if set(cube_errors) != expected:
            raise RuntimeError("Planning scene is missing cube geometry")
        robot_errors = {}
        # A replacement camera provider need only supply objects. Gazebo additionally
        # exposes robot links for a direct physics-pose versus TF consistency check.
        if isinstance(self.provider, GazeboStateProvider):
            for link in ["wrist_3_link"] + TOUCH_LINKS:
                tf = self.tf.lookup_transform("world", link, Time())
                xyz = tf.transform.translation
                real = self.provider.get(link).pose.position
                robot_errors[link] = float(np.linalg.norm([xyz.x-real.x, xyz.y-real.y, xyz.z-real.z]))
                if robot_errors[link] > 0.001:
                    raise RuntimeError(f"Gazebo / TF link pose disagree for {link}")
        self.record("scene_consistency", moveit_joint_error=joint_error,
                    gazebo_scene_cube_error_m=cube_errors, gazebo_tf_link_error_m=robot_errors)

    def allow_touch(self, name):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.service(GetPlanningScene, "/get_planning_scene", request).scene.allowed_collision_matrix
        self.original_acm = deepcopy(acm)
        for link in [name] + TOUCH_LINKS:
            if link not in acm.entry_names:
                acm.entry_names.append(link)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        i = acm.entry_names.index(name)
        for link in TOUCH_LINKS:
            j = acm.entry_names.index(link)
            acm.entry_values[i].enabled[j] = True
            acm.entry_values[j].enabled[i] = True
        self.apply_scene(PlanningScene(allowed_collision_matrix=acm))
        self.record("allow_touch", object=name, links=TOUCH_LINKS)

    def restore_touch(self):
        if self.original_acm is not None:
            self.apply_scene(PlanningScene(allowed_collision_matrix=self.original_acm))
            self.original_acm = None

    def attach(self, name):
        self.relative_object = np.linalg.inv(matrix(self.tcp_pose())) @ matrix(self.provider.get(name).pose)
        attached = AttachedCollisionObject(link_name="grasp_tcp", touch_links=TOUCH_LINKS)
        attached.object = self.cube_object(name, pose_from_matrix(self.relative_object), "grasp_tcp")
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        # MoveIt removes the world instance while applying ADD attachment. An
        # explicit REMOVE in the same diff would remove it twice and return false.
        self.apply_scene(scene)
        self.contact_object = name
        self.record("scene_attach", object=name, physical_grasp_confirmed=False)

    def detach(self, name):
        scene = PlanningScene()
        attached = AttachedCollisionObject(link_name="grasp_tcp")
        attached.object.id = name
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [attached]
        scene.world.collision_objects = [self.cube_object(name, self.provider.get(name).pose)]
        self.apply_scene(scene)
        self.contact_object = None
        self.relative_object = None
        self.record("scene_detach", object=name)

    def verify_holding(self, name, initial_z, duration=3.2):
        start_sim = self.get_clock().now().nanoseconds / 1e9
        deadline = time.monotonic() + 15.0
        measured = []
        while time.monotonic() < deadline:
            self.spin(0.1)
            pose = self.provider.get(name).pose
            relative = np.linalg.inv(matrix(self.tcp_pose())) @ matrix(pose)
            relative_error = float(np.linalg.norm(relative[:3, 3] - self.relative_object[:3, 3]))
            height = pose.position.z - initial_z
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start_sim
            measured.append({"sim_time": start_sim + elapsed, "cube": pose_data(pose),
                             "height_gain_m": height, "relative_error_m": relative_error})
            if height < 0.04 or relative_error > 0.008:
                self.record("holding_failed", samples=measured)
                raise RuntimeError(f"Physical grasp lost: lift {height:.4f}m, relative drift {relative_error:.4f}m")
            if elapsed >= duration:
                self.record("holding_pass", samples=measured, duration_sim_s=elapsed)
                return
        raise TimeoutError("Simulation clock stalled during grasp verification")

    def check_carried(self, name, min_height=0.12):
        pose = self.provider.get(name).pose
        relative = np.linalg.inv(matrix(self.tcp_pose())) @ matrix(pose)
        drift = float(np.linalg.norm(relative[:3, 3] - self.relative_object[:3, 3]))
        if pose.position.z < min_height or drift > 0.008:
            self.holding_verified = False
            raise RuntimeError(f"Physical object not carried: z={pose.position.z:.4f}, drift={drift:.4f}")
        self.record("carry_check", object=name, cube_z=pose.position.z, relative_error_m=drift)

    def result(self, skill, success, reason):
        result = SkillResult(success, skill, reason, self.held_object, self.holding_verified,
                             self.contact_object if not self.holding_verified else None,
                             bool(self.contact_object and not self.holding_verified))
        self.results.append(asdict(result))
        if not getattr(self, "compact_evidence", False):
            (self.evidence / "results.json").write_text(json.dumps(self.results, indent=2))
        self.record("skill_result", **asdict(result))
        return result

    def home(self):
        try:
            if self.contact_object:
                raise RuntimeError("Cannot home while an object is attached or a grasp is unresolved")
            self.sync_scene()
            self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], "home")
            self.snapshot("home")
            return self.result("home", True, "Arm returned to checked initial joint pose")
        except Exception as error:
            return self.result("home", False, str(error))

    def pick(self, name):
        try:
            if name not in self.config["cubes"]:
                raise ValueError(f"Unknown object {name}")
            if self.contact_object or self.held_object:
                raise RuntimeError("A grasp is already active or unresolved")
            self.active_object = name
            self.sync_scene()
            initial = self.provider.get(name).pose
            if abs(initial.position.z - 0.095) > 0.004:
                raise RuntimeError("Cube is not resting on the expected table surface")
            self.snapshot("initial")
            self.set_gripper(0.015)
            self.snapshot("gripper_open")
            target = deepcopy(initial)
            # +Z TCP points down; finger sliding axis remains along world X.
            target.orientation.x, target.orientation.y = 1.0, 0.0
            target.orientation.z, target.orientation.w = 0.0, 0.0
            target.position.z += self.tcp_offset_above_cube + 0.05
            self.move_pose(target, "above_cube", ik_seed=GRASP_IK_SEED)
            self.snapshot("above_cube")
            self.allow_touch(name)
            target.position.z -= 0.05
            self.cartesian(target, "descend_to_grasp")
            self.snapshot("grasp_pose")
            # At contact q~0.002m; q=0.0005 supplies physical preload while the
            # 0.0015m blocked-position error is within the unchanged goal tolerance.
            self.contact_object = name  # from this point an interrupted grasp needs recovery
            self.set_gripper(0.0005)
            self.snapshot("gripper_closed_on_cube")
            self.attach(name)
            target.position.z += 0.05
            self.cartesian(target, "lift_5cm")
            self.snapshot("cube_lifted")
            self.verify_holding(name, initial.position.z)
            self.held_object = name
            self.holding_verified = True
            self.snapshot("held_after_3_seconds")
            return self.result("pick", True, "Gazebo cube lifted >=4cm and stayed with TCP for >=3s")
        except Exception as error:
            self.holding_verified = False
            if not self.contact_object:
                try:
                    self.restore_touch()
                except Exception as cleanup_error:
                    self.record("touch_restore_failed", reason=str(cleanup_error))
            self.record("pick_failed", object=name, reason=str(error))
            # Stop here. Never transport on the strength of an attachment or action SUCCESS.
            return self.result("pick", False, str(error))

    def place(self, name, zone):
        try:
            if self.held_object != name or not self.holding_verified:
                raise RuntimeError("No physically verified held object; transport refused")
            if zone not in self.config["zones"]:
                raise ValueError(f"Unknown zone {zone}")
            self.check_carried(name)
            self.verify_zone_empty(name, zone)
            destination = self.config["zones"][zone]["placement_cube_center"]
            # Use measured cube/TCP offset, rather than assuming a perfect grasp.
            target = self.tcp_pose()
            cube_offset = matrix(target)[:3, :3] @ self.relative_object[:3, 3]
            target.position.x = destination[0] - float(cube_offset[0])
            target.position.y = destination[1] - float(cube_offset[1])
            target.position.z = destination[2] + 0.05 - float(cube_offset[2])
            self.cartesian(target, "carry_above_zone")
            self.check_carried(name)
            self.snapshot("above_zone")
            target.position.z -= 0.048  # release 2mm above table; avoid attached/table contact
            self.cartesian(target, "lower_to_zone")
            self.snapshot("lowered_at_zone")
            self.set_gripper(0.015)
            self.spin(0.8)
            self.detach(name)
            self.held_object = None
            self.holding_verified = False
            target.position.z += 0.06
            self.cartesian(target, "retreat_after_release")
            self.restore_touch()
            self.sync_scene()
            self.spin(1.0)
            actual = self.provider.get(name).pose
            half = self.config["zones"][zone]["size_xy"]
            size = self.config["cubes"][name]["size"]
            if (abs(actual.position.x-destination[0]) > (half[0]-size[0])/2 or
                abs(actual.position.y-destination[1]) > (half[1]-size[1])/2 or
                abs(actual.position.z-destination[2]) > 0.003):
                raise RuntimeError(f"Released cube is outside {zone}: {pose_data(actual)}")
            self.snapshot("released_and_retreated")
            return self.result("place", True, f"Gazebo cube settled fully inside {zone}")
        except Exception as error:
            self.record("place_failed", object=name, zone=zone, reason=str(error))
            return self.result("place", False, str(error))

    def verify_zone_empty(self, name, zone):
        destination = self.config["zones"][zone]["placement_cube_center"]
        width = self.config["zones"][zone]["size_xy"]
        occupied = []
        for other, spec in self.config["cubes"].items():
            if other == name:
                continue
            pose = self.provider.get(other).pose
            if (abs(pose.position.x - destination[0]) < (width[0] + spec["size"][0])/2 and
                abs(pose.position.y - destination[1]) < (width[1] + spec["size"][1])/2):
                occupied.append(other)
        self.record("zone_empty_check", object=name, zone=zone, occupied_by=occupied)
        if occupied:
            raise RuntimeError(f"Target zone {zone} is occupied by {occupied}")
