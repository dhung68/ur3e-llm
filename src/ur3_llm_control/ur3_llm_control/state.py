"""Read-only state-provider boundary; a camera provider can implement it later."""
from copy import deepcopy
from dataclasses import dataclass
import time
from typing import Protocol

from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage


@dataclass
class ObjectState:
    name: str
    pose: Pose
    observed_wall_time: float


class StateProvider(Protocol):
    def get(self, name: str) -> ObjectState: ...


class GazeboStateProvider:
    """Model poses from Gazebo, never from the planning-scene attachment."""
    def __init__(self, node):
        self.objects = {}
        self.subscription = node.create_subscription(
            TFMessage, "/bai2/gazebo_poses", self._update, 10
        )

    def _update(self, message):
        received = time.monotonic()
        for transform in message.transforms:
            # Model names are unique. Generic link entries are intentionally ignored.
            name = transform.child_frame_id
            if name.endswith("_cube") or name in ("ur", "work_table", "wrist_3_link", "left_finger", "right_finger"):
                pose = Pose()
                p = transform.transform.translation
                pose.position.x, pose.position.y, pose.position.z = p.x, p.y, p.z
                pose.orientation = deepcopy(transform.transform.rotation)
                self.objects[name] = ObjectState(name, pose, received)

    def get(self, name):
        if name not in self.objects:
            raise RuntimeError(f"No Gazebo observation for {name}")
        result = self.objects[name]
        if time.monotonic() - result.observed_wall_time > 1.0:
            raise RuntimeError(f"Stale Gazebo observation for {name}")
        return deepcopy(result)
