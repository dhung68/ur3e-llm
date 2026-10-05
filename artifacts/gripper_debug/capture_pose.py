import json
import re
import subprocess
import sys
from pathlib import Path

import rclpy
from probe import Probe

rclpy.init()
node = Probe()
node.settle(0.5)
raw = subprocess.run(['ign', 'topic', '-e', '-t', '/world/bai2/pose/info', '-n', '1'],
                     capture_output=True, text=True, timeout=8, check=True).stdout
Path(sys.argv[1] + '.txt').write_text(raw)
node.settle(0.1)
poses = {}
for m in re.finditer(r'pose \{\n  name: "(wrist_3_link|left_finger|right_finger)"\n(.*?)\n\}', raw, re.S):
    body = m[2]
    def vector(key, fields):
        block = re.search(key + r' \{(.*?)\}', body, re.S)[1]
        return [float(re.search(r'\b' + f + r': ([^\s]+)', block)[1])
                if re.search(r'\b' + f + r': ([^\s]+)', block) else 0.0 for f in fields]
    poses[m[1]] = {'position': vector('position', 'xyz'), 'quaternion': vector('orientation', 'xyzw')}
parent = poses['wrist_3_link']
x, y, z, w = parent['quaternion']
# Columns of the wrist rotation are its local axes expressed in world.
axes = [[1-2*(y*y+z*z), 2*(x*y+z*w), 2*(x*z-y*w)],
        [2*(x*y-z*w), 1-2*(x*x+z*z), 2*(y*z+x*w)],
        [2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y)]]
report = {'poses': poses, 'joint_states': node.joints, 'fingers': {}}
for side, sign in [('left', 1), ('right', -1)]:
    delta = [p-b for p,b in zip(poses[side+'_finger']['position'], parent['position'])]
    local = [sum(a*d for a,d in zip(axis, delta)) for axis in axes]
    q = sign*local[0] - 0.018
    ros_q = node.joints[side+'_finger_joint']
    report['fingers'][side] = {'wrist_local_position': local, 'q_from_gazebo_pose': q,
                               'q_from_joint_states': ros_q, 'difference': abs(q-ros_q)}
print(json.dumps(report, indent=2))
Path(sys.argv[1] + '.json').write_text(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
