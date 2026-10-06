"""Focused occlusion, retained occupancy and checkpoint policy checks."""
import time
from types import SimpleNamespace
from unittest.mock import Mock,patch
import pytest
from geometry_msgs.msg import Pose
from ur3_llm_control.bai3_state import CameraStateProvider
from ur3_llm_control.state import ObjectState
from ur3_llm_control.bai3_skills import CameraSkills


def provider():
 p=object.__new__(CameraStateProvider);p.received=time.monotonic()
 p.message={'objects':{},'errors':{'green_cube':'missing/occluded'}}
 pose=Pose();pose.position.x=.38;pose.position.y=0.;pose.position.z=.095;pose.orientation.w=1.
 p.last_valid={'green_cube':ObjectState('green_cube',pose,time.monotonic()-5)}
 return p


def test_occluded_obstacle_not_empty_or_freshened():
 p=provider();o,cached=p.obstacle('green_cube')
 assert cached and o.pose.position.x==.38 and time.monotonic()-o.observed_wall_time>4
 with pytest.raises(RuntimeError):p.get('green_cube')
 p.message={'error':'Camera stale'}
 with pytest.raises(RuntimeError,match='stale'):p.obstacle('green_cube')


def test_occluded_occupied_zone_rejected():
 p=provider();s=SimpleNamespace(provider=p,record=Mock(),config={'cubes':{'green_cube':{'size':[.03]*3},'blue_cube':{'size':[.03]*3}},
     'zones':{'zone_b':{'placement_cube_center':[.38,0,.095],'size_xy':[.08,.08]}}})
 with pytest.raises(RuntimeError,match='occupied/uncertain'):CameraSkills.verify_zone_empty(s,'blue_cube','zone_b')


def test_motion_can_occlude_object_but_still_uses_joint_freshness():
 future=Mock();future.done.side_effect=[False,True];future.result.return_value=SimpleNamespace(status=4,result=SimpleNamespace(error_code=SimpleNamespace(val=1)))
 handle=Mock(accepted=True);handle.get_result_async.return_value=future
 p=provider();p.get=Mock(side_effect=RuntimeError('missing/occluded'))
 s=SimpleNamespace(wait=Mock(return_value=handle),provider=p,state=Mock(),record=Mock(),spin=Mock())
 with patch('rclpy.ok',return_value=True),patch('rclpy.spin_once'):
  CameraSkills.action(s,Mock(),object(),'carry')
 p.get.assert_not_called();s.state.assert_called_once()


def test_free_target_preserves_padded_occluded_collision():
 p=provider();s=SimpleNamespace(provider=p,record=Mock(),apply_scene=Mock(),cube_object=Mock(),
    config={'cubes':{'green_cube':{'size':[.03]*3},'blue_cube':{'size':[.03]*3}},
            'zones':{'temp_1':{'placement_cube_center':[.16,-.14,.095],'size_xy':[.04,.04]}}})
 from moveit_msgs.msg import CollisionObject
 from shape_msgs.msg import SolidPrimitive
 s.cube_object.return_value=CollisionObject(id='green_cube',operation=CollisionObject.ADD,
      primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.03]*3)])
 CameraSkills.verify_zone_empty(s,'blue_cube','temp_1')
 obj=s.apply_scene.call_args[0][0].world.collision_objects[0]
 assert obj.id=='green_cube' and obj.operation==CollisionObject.ADD
 assert abs(obj.primitives[0].dimensions[0]-.09)<1e-9
