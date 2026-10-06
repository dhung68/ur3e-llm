"""Assignment 3: camera-guided physical skills, every actuator through MoveIt."""
from copy import deepcopy
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from .skills import Skills, FINGERS, ARM, GRASP_IK_SEED
from .bai3_state import CameraStateProvider, free_temporaries, with_temporaries


def scene_config():
    return json.loads((Path(get_package_share_directory('hri_bai3_environment'))/'config/scene.json').read_text())


class CameraSkills(Skills):
    def __init__(self, evidence):
        super().__init__(evidence, compact_evidence=True, node_name='bai3_task',
                         scene_config=scene_config(), provider_factory=CameraStateProvider)

    def set_gripper(self, position):
        import time
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint
        goal=MoveGroup.Goal()
        goal.request.group_name='gripper'
        goal.request.num_planning_attempts=5;goal.request.allowed_planning_time=15.
        goal.request.max_velocity_scaling_factor=.3;goal.request.max_acceleration_scaling_factor=.3
        goal.request.start_state.is_diff=True
        goal.request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(
            joint_name=name,position=position,tolerance_above=.00001,tolerance_below=.00001,weight=1.) for name in FINGERS])]
        goal.planning_options.planning_scene_diff.is_diff=True
        goal.planning_options.planning_scene_diff.robot_state.is_diff=True
        response=self.action(self.move,goal,f'gripper_moveit_{position}',30.)
        # The controller's original 2mm tolerance can finish before finger settling.
        # Observe convergence; never retransmit motion or widen controller tolerances.
        deadline=time.monotonic()+2.
        while True:
            errors={name:abs(self.joints[name]-position) for name in FINGERS}
            if max(errors.values())<=.002:break
            if time.monotonic()>=deadline:
                raise RuntimeError(f'gripper_moveit_{position}: measured finger target not reached {errors}')
            self.spin(.1)
        self.record('joint_target_check',label=f'gripper_moveit_{position}',errors=errors)
        return response

    def available_temporaries(self):
        self.spin(.2)
        positions = self.provider.snapshot(self.config['cubes'])
        geometric = free_temporaries(positions, self.config)
        self.sync_scene()
        reachable = {}
        for name, xyz in geometric.items():
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'ur_grasp'
            req.ik_request.ik_link_name = 'grasp_tcp'
            req.ik_request.avoid_collisions = True
            req.ik_request.timeout.sec = 2
            req.ik_request.robot_state = self.state()
            for joint, q in zip(ARM, GRASP_IK_SEED):
                js = req.ik_request.robot_state.joint_state
                js.position[js.name.index(joint)] = q
            target = PoseStamped()
            target.header.frame_id = 'world'
            target.pose.position.x, target.pose.position.y = xyz[:2]
            target.pose.position.z = xyz[2] + self.tcp_offset_above_cube + .05
            target.pose.orientation.x = 1.
            req.ik_request.pose_stamped = target
            result = self.service(GetPositionIK, '/compute_ik', req)
            if result.error_code.val == 1: reachable[name] = xyz
        self.config = with_temporaries(self.config, reachable)
        return positions, reachable

    def action(self, client, goal, label, timeout=90., require_success=True):
        """Camera checks at stable grasp/hold/place checkpoints; occlusion is allowed.

        No new goal is sent until a timed-out/failed action has a terminal result.
        Cancellation failures retain the original camera error and recovery state.
        """
        import time
        import numpy as np
        import rclpy
        from .skills import matrix, pose_data
        handle = self.wait(client.send_goal_async(goal), 10.)
        if not handle.accepted:
            self.record('action', label=label, accepted=False)
            raise RuntimeError(f'{label}: goal rejected')
        future = handle.get_result_async()
        deadline = time.monotonic()+timeout
        samples=[];last_sample=0.
        try:
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=.05)
                if time.monotonic()>deadline: raise TimeoutError(f'{label}: action timeout')
                if time.monotonic()-self.provider.received > 1 or self.provider.message.get('error'):
                    raise RuntimeError(self.provider.message.get('error', 'Camera state stale during motion'))
                # Camera object checks occur at stable observation checkpoints.
                # Occlusion or different view timestamps during motion do not
                # replace measurements with attachment/Gazebo poses.
                self.state()  # measured joint freshness remains mandatory
            response=future.result()
        except Exception as original:
            self.record('action_observation_failed',label=label,reason=str(original))
            cancel=handle.cancel_goal_async()
            try:
                cancellation=self.wait(cancel, 15.)
                self.record('action_cancelled',label=label,reason=str(original),cancellation_return_code=cancellation.return_code)
                self.wait(future, 30.)
            except Exception as cancellation_error:
                self.record('action_terminal_unconfirmed',label=label,reason=str(cancellation_error))
                self.motion_unresolved=True
            raise RuntimeError(f'{label}: {original}') from original
        code=response.result.error_code.val
        self.record('action',label=label,accepted=True,status=response.status,error_code=code,samples=samples)
        if require_success and (response.status != 4 or code != 1):
            raise RuntimeError(f'{label}: status {response.status}, error {code}')
        self.spin(.3)
        return response

    def sync_scene(self):
        """Retain occluded collision geometry; only home may clear the occlusion.

        Picks/LLM validation still require a full fresh camera snapshot. After
        release, this partial update permits empty-hand retreat/observation home.
        It never infers that an occluded object disappeared or a region is free.
        """
        import time
        from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        measured={};missing=[]
        for name in self.config['cubes']:
            if name == self.contact_object: continue
            try: measured[name]=self.provider.get(name).pose
            except RuntimeError as error:
                if 'missing/occluded' not in str(error): raise
                missing.append(name)
        if not missing:
            return super().sync_scene()
        if self.contact_object or self.held_object:
            raise RuntimeError(f'Full camera scene required during grasp: {missing}')
        if time.monotonic()-self.provider.received > 1 or self.provider.message.get('error'):
            raise RuntimeError('Fresh partial camera frame required for observation home')
        req=GetPlanningScene.Request()
        req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        observed=self.service(GetPlanningScene,'/get_planning_scene',req).scene
        ids={obj.id for obj in observed.world.collision_objects}
        if observed.robot_state.attached_collision_objects or not set(self.config['cubes']).issubset(ids):
            raise RuntimeError('Cannot observe home: missing retained collision geometry or attachment')
        diff=PlanningScene()
        diff.world.collision_objects=[self.cube_object(name,pose) for name,pose in measured.items()]
        self.apply_scene(diff)
        self.record('partial_scene_for_observation_home', retained_occluded=missing,
                    refreshed_camera_objects=list(measured), full_scene_verified=False)

    def prepare_place_visibility(self):
        self.check_carried(self.held_object)

    def check_carried(self,name,min_height=.12):
        try:return super().check_carried(name,min_height)
        except RuntimeError as error:
            if not any(x in str(error) for x in ('missing/occluded','partial view','quality/coordinates')):raise
            if not self.holding_verified or getattr(self,'observation_used',False):raise
        self.observation_used=True
        target=self.tcp_pose();target.position.y-=.06;target.position.z+=.025
        if target.position.z>.3 or target.position.y<-.23:
            raise RuntimeError('No bounded camera observation pose; stop without blind retry')
        self.record('camera_observation_motion',object=name,offset_y_m=-.06,offset_z_m=.025)
        self.cartesian(target,'observe_held_object')
        self.spin(.4)
        return super().check_carried(name,min_height)

    def verify_zone_empty(self,name,zone):
        import time
        destination=self.config['zones'][zone]['placement_cube_center']
        width=self.config['zones'][zone]['size_xy'];occupied=[];retained={}
        for other,spec in self.config['cubes'].items():
            if other==name:continue
            observed,cached=self.provider.obstacle(other)
            p=observed.pose.position
            margin=.03 if cached else 0.
            if cached:retained[other]={'last_camera_position':[p.x,p.y,p.z],
                                     'age_s':time.monotonic()-observed.observed_wall_time,'extra_margin_m':margin}
            if abs(p.x-destination[0])<(width[0]+spec['size'][0])/2+margin and abs(p.y-destination[1])<(width[1]+spec['size'][1])/2+margin:
                occupied.append(other)
        self.record('zone_empty_check',zone=zone,occupied_by=occupied,retained_occluded_obstacles=retained)
        if occupied:raise RuntimeError(f'Target {zone} occupied/uncertain: {occupied}; observe again first')
        if retained:
            from moveit_msgs.msg import PlanningScene
            diff=PlanningScene()
            for other in retained:
                observed,_=self.provider.obstacle(other)
                obj=self.cube_object(other,observed.pose)
                obj.primitives[0].dimensions[0]+=.06
                obj.primitives[0].dimensions[1]+=.06
                diff.world.collision_objects.append(obj)
            self.apply_scene(diff)
            self.record('occluded_collision_geometry_retained',objects=list(retained),padding_xy_m=.03)


    def pick(self,name):
        self.observation_used=False
        return super().pick(name)

    def service(self, typ, name, request, timeout=15.):
        client=self.create_client(typ,name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'Missing service {name}')
            # New DDS request/response endpoints need a callback/discovery turn.
            self.spin(.1)
            return self.wait(client.call_async(request),timeout)
        finally: self.destroy_client(client)

    def result(self, skill, success, reason):
        return super().result(skill,success,reason.replace('Gazebo cube','Camera RGB-D cube'))
