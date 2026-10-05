#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

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
        self.samples = []
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/gripper_controller/controller_state', self.on_state, 10)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_state(self, msg):
        self.samples.append({'t': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                             'reference': list(msg.reference.positions),
                             'feedback': list(msg.feedback.positions),
                             'error': list(msg.error.positions)})

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
        handle = self.wait(client.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError('Goal rejected')
        try:
            result = self.wait(handle.get_result_async(), duration + 15)
        except TimeoutError:
            self.wait(handle.cancel_goal_async(), 5)
            raise
        self.settle()
        positions = [self.joints[n] for n in names]
        record = {'controller': controller, 'target': target, 'status': result.status,
                  'error_code': result.result.error_code,
                  'error_string': result.result.error_string,
                  'positions': positions,
                  'abs_error': [abs(a-b) for a, b in zip(positions, target)],
                  'samples': self.samples}
        print(json.dumps({k: v for k, v in record.items() if k != 'samples'}), flush=True)
        client.destroy()
        return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--wrist', type=float)
    parser.add_argument('--cycles', type=int, default=3)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    node.settle(1)
    records = []
    try:
        if args.wrist is not None:
            names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            target = [node.joints[n] for n in names]
            target[-1] = args.wrist
            records.append(node.trajectory('joint_trajectory_controller', names, target, 5))
        for i in range(args.cycles):
            for target in ([0.0, 0.0], [0.015, 0.015]):
                records.append(node.trajectory('gripper_controller',
                                               ['left_finger_joint', 'right_finger_joint'], target, 3))
    finally:
        Path(args.output).write_text(json.dumps(records, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
