#!/usr/bin/env python3
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from probe import Probe


def service(node, typ, name, request):
    client = node.create_client(typ, name)
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError(f'Missing {name}')
    return node.wait(client.call_async(request), 15)


rclpy.init()
node = Probe()
node.settle(1)
report = {}
try:
    request = GetParameters.Request(names=['robot_description'])
    description = service(node, GetParameters, '/move_group/get_parameters', request).values[0].string_value
    robot = ET.fromstring(description)
    for name in ['gripper_base', 'left_finger', 'right_finger']:
        link = robot.find(f"link[@name='{name}']")
        assert link is not None and link.find('collision') is not None
    report['moveit_gripper_collision_links'] = ['gripper_base', 'left_finger', 'right_finger']
    request = GetPlanningScene.Request()
    request.components.components = (PlanningSceneComponents.ROBOT_STATE |
                                     PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                     PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    scene = service(node, GetPlanningScene, '/get_planning_scene', request).scene
    report['scene_objects'] = [obj.id for obj in scene.world.collision_objects]
    state = scene.robot_state.joint_state
    report['moveit_joint_positions'] = dict(zip(state.name, state.position))
    report['ros_joint_positions'] = node.joints
    req = GetStateValidity.Request()
    req.robot_state = scene.robot_state
    req.group_name = 'ur_manipulator'
    valid = service(node, GetStateValidity, '/check_state_validity', req)
    report['state_valid'] = valid.valid
    report['contacts'] = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
    print(json.dumps(report), flush=True)
    client = ActionClient(node, MoveGroup, '/move_action')
    assert client.wait_for_server(timeout_sec=10)
    goal = MoveGroup.Goal()
    goal.request.group_name = 'ur_manipulator'
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = 0.1
    goal.request.max_acceleration_scaling_factor = 0.1
    goal.request.start_state.is_diff = True
    constraints = Constraints()
    names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    targets = [node.joints[n] for n in names]
    targets[0] += 0.1
    targets[-1] = float(sys.argv[2]) if len(sys.argv) > 2 else -1.57079632679
    for name, target in zip(names, targets):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=target, tolerance_above=0.001,
            tolerance_below=0.001, weight=1.0))
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    handle = node.wait(client.send_goal_async(goal), 10)
    assert handle.accepted
    try:
        result = node.wait(handle.get_result_async(), 60)
    except TimeoutError:
        node.wait(handle.cancel_goal_async(), 5)
        raise
    node.settle(0.5)
    continuous = {j.attrib['name'] for j in robot.findall('joint') if j.get('type') == 'continuous'}
    errors = [node.joints[n] - t for n,t in zip(names,targets)]
    errors = [abs(math.atan2(math.sin(e), math.cos(e))) if n in continuous else abs(e)
              for n,e in zip(names,errors)]
    report['arm'] = {'status': result.status, 'moveit_error_code': result.result.error_code.val,
                     'target': targets, 'actual': [node.joints[n] for n in names],
                     'abs_error': errors,
                     'planned_points': len(result.result.planned_trajectory.joint_trajectory.points)}
    print(json.dumps(report['arm']), flush=True)
    assert result.status == 4 and result.result.error_code.val == 1
finally:
    Path(sys.argv[1]).write_text(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
