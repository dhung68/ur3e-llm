"""Camera state only. Missing measurements never imply free space."""
from copy import deepcopy
from collections import deque
from functools import wraps
from threading import Event, RLock, Thread
import json
import math
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from .state import ObjectState
from .planning import PlanError


def _locked(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._state_lock:
            return method(self, *args, **kwargs)
    return call


class CameraStateProvider:
    def __init__(self, node, independent=False):
        self._state_lock = RLock()
        self._receiver_error = None
        self._receiver_stop = Event()
        self._receiver_executor = None
        self._receiver_thread = None
        self.node = node
        self.received = 0.
        self.message = {}
        self.last_valid = {}
        self.update_count = 0
        self.last_stamp = None
        self.receive_history = deque(maxlen=32)
        if independent:
            from rclpy.node import Node
            from rclpy.parameter import Parameter
            from rclpy.executors import SingleThreadedExecutor
            # A separate callback group on the task's single executor would
            # still wait behind its clock/TF/action callbacks. This node owns
            # only the camera stream and its ROS clock, pumped continuously.
            self.node = Node('bai3_camera_receiver', context=node.context,
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self._receiver_executor = SingleThreadedExecutor(context=node.context)
            self._receiver_executor.add_node(self.node)
        self.subscription = self.node.create_subscription(String, '/bai3/table_state', self.update, 1)
        if independent:
            self._receiver_thread = Thread(target=self._receive, name='bai3_camera_receiver', daemon=True)
            self._receiver_thread.start()

    def _receive(self):
        try:
            while self.node.context.ok() and not self._receiver_stop.is_set():
                self._receiver_executor.spin_once(timeout_sec=.01)
        except Exception as error:
            if not self._receiver_stop.is_set():
                self._receiver_error = str(error) or type(error).__name__

    def close(self):
        if self._receiver_executor is None:
            return
        self._receiver_stop.set()
        self._receiver_executor.wake()
        self._receiver_thread.join(timeout=3.)
        if self._receiver_thread.is_alive():
            raise RuntimeError('Camera receiver did not stop; node destruction refused')
        self._receiver_executor.remove_node(self.node)
        self._receiver_executor.shutdown()
        self.node.destroy_node()
        self._receiver_executor = None

    @_locked
    def update(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('source') != 'rgbd' or data.get('frame_id') != 'world':
                raise ValueError('Unexpected perception source/frame')
            if not data.get('error'):
                stamp = data.get('stamp')
                if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                    raise ValueError('Invalid camera capture timestamp')
                if self.last_stamp is not None and stamp <= self.last_stamp:
                    # Replayed/out-of-order frames must not refresh the stream.
                    return
                self.last_stamp = stamp
            self.message = data
            self.received = time.monotonic()
            self.update_count += 1
            self.receive_history.append({'wall_time': time.time(), 'stamp': data.get('stamp'),
                'capture_age_s': self.received-data['capture_monotonic_s']
                    if isinstance(data.get('capture_monotonic_s'), (int,float)) else None,
                'error': data.get('error')})
            if not data.get('error'):
                for name,item in data.get('objects',{}).items():
                    try:
                        state=self.get(name)
                    except (RuntimeError,KeyError,TypeError,ValueError):continue
                    state.observed_wall_time=data.get('capture_monotonic_s',self.received-item.get('age_s',0.))
                    self.last_valid[name]=deepcopy(state)
        except (ValueError, TypeError):
            self.message = {'error': 'Invalid perception message'}

    @_locked
    def check_fresh(self):
        if self._receiver_error:
            raise RuntimeError(f'Camera receiver failed: {self._receiver_error}')
        if time.monotonic()-self.received > 1.0:
            raise RuntimeError('Camera state stale/unavailable; no simulator fallback')
        if self.message.get('error'):
            raise RuntimeError(self.message['error'])
        # Same-host first RGB-D callback time includes processing/transport wait.
        # Header age is checked separately in ROS time, without mixing clocks.
        captured = self.message.get('capture_monotonic_s')
        if captured is not None:
            if not isinstance(captured, (int, float)) or not math.isfinite(captured):
                raise RuntimeError('Invalid camera capture clock')
            if not 0 <= time.monotonic()-captured <= 1.0:
                raise RuntimeError('Camera RGB-D capture stale during motion')
        image_age = self.image_age_sim()
        if not isinstance(image_age,(int,float)) or not math.isfinite(image_age) or image_age>1:
            raise RuntimeError('Camera image header stale in simulation time')

    @_locked
    def image_age_sim(self):
        age=self.message.get('image_age_sim_s',0.)
        if isinstance(self.message.get('stamp'),(int,float)) and hasattr(self.node,'get_clock'):
            age=max(age,self.node.get_clock().now().nanoseconds/1e9-self.message['stamp'])
        return age

    @_locked
    def diagnostics(self, include_history=False):
        now = time.monotonic()
        captured = self.message.get('capture_monotonic_s')
        try: image_age=self.image_age_sim()
        except Exception: image_age=None  # Diagnostics must not block cancellation.
        result = {'callback_age_s': now-self.received, 'stamp': self.message.get('stamp'),
                'capture_age_s': now-captured if isinstance(captured,(int,float)) else None,
                'image_age_sim_s': image_age,
                'update_count': getattr(self,'update_count',0), 'error': self.message.get('error'),
                'receiver_error': self._receiver_error}
        if include_history:
            result['receive_history'] = list(self.receive_history)
        return result

    @_locked
    def get(self, name):
        self.check_fresh()
        # A message contains only this frame's measured objects. No hidden tracker.
        item = self.message.get('objects', {}).get(name)
        if item is None:
            raise RuntimeError(f"Camera cannot observe {name}: {self.message.get('errors', {}).get(name, 'missing')}")
        if item.get('age_s', 0)+time.monotonic()-self.received > 1.:
            raise RuntimeError(f'Camera measurement stale for {name}')
        xyz = item['position']
        if item.get('quality', 0) < .6 or len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
            raise RuntimeError(f'Camera quality/coordinates invalid for {name}')
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = item['quaternion']
        return ObjectState(name, pose, self.received)

    @_locked
    def obstacle(self,name):
        """Occluded static obstacle stays occupied at its last camera measurement.

        This cache is NOT a pick/held-object pose or an empty-space observation.
        Stale/error camera streams and ambiguous colors are never accepted.
        """
        try:return self.get(name),False
        except RuntimeError as error:
            text=str(error)
            if not any(x in text for x in ('missing/occluded','partial view','quality/coordinates')):raise
            self.check_fresh()
            if name not in self.last_valid:raise RuntimeError(f'Unknown obstacle {name}; cannot declare space free')
            return deepcopy(self.last_valid[name]),True

    @_locked
    def snapshot(self, names):
        # Require every object: an occluded cube cannot be removed from occupancy.
        return {name: [s.pose.position.x, s.pose.position.y, s.pose.position.z]
                for name in names for s in [self.get(name)]}


def overlaps(a, b, half_x, half_y):
    return abs(a[0]-b[0]) < half_x and abs(a[1]-b[1]) < half_y


def occupants(positions, config, destination, exclude=None):
    d = config['zones'][destination]
    return [name for name, p in positions.items() if name != exclude and
            overlaps(p, d['placement_cube_center'], (d['size_xy'][0]+.03)/2,
                     (d['size_xy'][1]+.03)/2)]


def free_temporaries(positions, config):
    table = config['table']
    available = {}
    for name, xyz in config['temporary_candidates'].items():
        if any(abs(xyz[i]-table['center'][i]) > table['size'][i]/2-.035 for i in (0, 1)):
            continue
        # Cube footprint + 25mm clearance for fingers/uncertainty.
        if any(overlaps(xyz, p, .055, .055) for p in positions.values()):
            continue
        # Buffers must not overlap a semantic destination region.
        if any(overlaps(xyz, z['placement_cube_center'], .07, .07) for z in config['zones'].values()):
            continue
        available[name] = deepcopy(xyz)
    return available


def with_temporaries(config, candidates):
    result = deepcopy(config)
    for name, xyz in candidates.items():
        result['zones'][name] = {'placement_cube_center': xyz, 'size_xy': [.04, .04]}
    return result


def validate_bai3(plan, config, positions, held=None):
    """Simulate full occupancy transitions, including moving blockers to buffers."""
    if type(plan) is not dict or set(plan) != {'steps'} or type(plan['steps']) is not list or not 1 <= len(plan['steps']) <= 24:
        raise PlanError('Expected only steps with 1..24 skills')
    if set(positions) != set(config['cubes']):
        raise PlanError('All five fresh camera objects are required')
    if held is not None:
        raise PlanError('Existing/unresolved grasp')
    if plan['steps'][0] != {'skill': 'home'} or plan['steps'][-1] != {'skill': 'home'}:
        raise PlanError('Plan must begin and end home for an unobstructed camera observation')
    state = deepcopy(positions)
    keys = {'home': {'skill'}, 'pick': {'skill','object'}, 'place': {'skill','object','zone'}}
    previous = None
    for i, step in enumerate(plan['steps'], 1):
        if type(step) is not dict or type(step.get('skill')) is not str:
            raise PlanError(f'Step {i}: invalid skill')
        skill = step['skill']
        if skill not in keys or set(step) != keys[skill]:
            raise PlanError(f'Step {i}: invalid skill/parameters')
        if skill != 'home' and (type(step['object']) is not str or step['object'] not in config['cubes']):
            raise PlanError(f'Step {i}: unknown object')
        if previous == 'place' and skill != 'home':
            raise PlanError(f'Step {i}: home required after release to refresh camera/scene')
        if skill == 'home':
            if held: raise PlanError(f'Step {i}: home while holding')
        elif skill == 'pick':
            if held: raise PlanError(f'Step {i}: hand occupied')
            held = step['object']
        else:
            zone = step['zone']
            if type(zone) is not str or zone not in config['zones']:
                raise PlanError(f'Step {i}: unavailable zone/temporary location')
            if held != step['object']:
                raise PlanError(f'Step {i}: place before matching pick')
            occupied = occupants(state, config, zone, exclude=held)
            if occupied:
                raise PlanError(f'Step {i}: {zone} occupied by {occupied}; move blocker first')
            state[held] = deepcopy(config['zones'][zone]['placement_cube_center'])
            held = None
        previous = skill
    if held: raise PlanError('Plan ends holding an object')
    return plan
