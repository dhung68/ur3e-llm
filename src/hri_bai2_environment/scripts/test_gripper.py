#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path
from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class Probe(Node):
    def __init__(self):
        super().__init__('gripper_probe')
        self.joints = None
        self.joint_samples = []
        self.samples = []
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/gripper_controller/controller_state', self.on_state, 10)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.joint_samples.append(self.joints.copy())

    def on_state(self, msg):
        self.samples.append({'t': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                             'reference': list(msg.reference.positions),
                             'feedback': list(msg.feedback.positions),
                             'error': list(msg.error.positions),
                             'velocity': list(msg.feedback.velocities),
                             'force_command': list(msg.output.effort)})

    def wait(self, future, seconds):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError('ROS future timed out')
        return future.result()

    def settle(self, seconds=0.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def trajectory(self, controller, names, target, duration):
        client = ActionClient(self, FollowJointTrajectory,
                              f'/{controller}/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('No action server')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = target
        p.velocities = [0.0] * len(names)
        p.time_from_start.sec = duration
        goal.trajectory.points = [p]
        self.samples = []
        self.joint_samples = []
        handle = self.wait(client.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError('Goal rejected')
        try:
            result = self.wait(handle.get_result_async(), duration + 15)
        except TimeoutError:
            self.wait(handle.cancel_goal_async(), 5)
            raise
        at_result = [self.joints[n] for n in names]
        self.settle()
        positions = [self.joints[n] for n in names]
        record = {'controller': controller, 'target': target, 'status': result.status,
                  'error_code': result.result.error_code,
                  'error_string': result.result.error_string,
                  'positions': positions,
                  'positions_at_result': at_result,
                  'abs_error_at_result': [abs(a-b) for a, b in zip(at_result, target)],
                  'abs_error': [abs(a-b) for a, b in zip(positions, target)],
                  'observed_min': [min(s[n] for s in self.joint_samples) for n in names],
                  'observed_max': [max(s[n] for s in self.joint_samples) for n in names],
                  'samples': self.samples}
        print(json.dumps({k: v for k, v in record.items() if k != 'samples'}), flush=True)
        client.destroy()
        return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/gripper_test.json')
    parser.add_argument('--cycles', type=int, default=3)
    args = parser.parse_args()
    if args.cycles < 3:
        parser.error('Use at least three complete cycles')
    rclpy.init()
    node = Probe()
    node.settle(1)
    records = []
    try:
        names = ['left_finger_joint', 'right_finger_joint']
        if node.joints is None or any(n not in node.joints for n in names):
            raise RuntimeError('No fresh finger states on /joint_states')
        if node.count_publishers('/joint_states') != 1:
            raise RuntimeError('Expected exactly one /joint_states publisher')
        # Start closed so every measured cycle includes a real opening motion.
        targets = [[0.0, 0.0]]
        for i in range(args.cycles):
            targets.extend([[0.015, 0.015], [0.0, 0.0]])
        for target in targets:
            record = node.trajectory('gripper_controller', names, target, 3)
            records.append(record)
            assert record['status'] == GoalStatus.STATUS_SUCCEEDED, 'Action did not succeed'
            assert record['error_code'] == 0, 'Controller rejected/aborted trajectory'
            assert max(record['abs_error_at_result'] + record['abs_error']) <= 0.002, 'Goal error > 2 mm'
            # Numerical contact solver penetration is measured separately from
            # goal tolerance; allow at most one micrometre at the hard stop.
            assert min(record['observed_min']) >= -1e-6, 'Lower limit exceeded'
            assert max(record['observed_max']) <= 0.020001, 'Upper limit exceeded'
        print(f'PASS: {args.cycles} open-close cycles, action SUCCESS, goal errors <= 0.002 m', flush=True)
    finally:
        Path(args.output).write_text(json.dumps(records, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
