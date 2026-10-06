from types import SimpleNamespace
from unittest.mock import Mock
from ur3_llm_control.bai3_skills import CameraSkills
from ur3_llm_control.skills import FINGERS


def test_gripper_tight_goal_uses_moveit_and_keeps_2mm_measured_check():
 s=SimpleNamespace(move=object(),joints=dict.fromkeys(FINGERS,.015),action=Mock(),record=Mock())
 CameraSkills.set_gripper(s,.015)
 goal=s.action.call_args[0][1]
 assert goal.request.group_name=='gripper'
 assert all(c.tolerance_above==.00001 for c in goal.request.goal_constraints[0].joint_constraints)
 assert s.action.call_count==1


def test_settling_observes_without_resending_motion():
 s=SimpleNamespace(move=object(),joints=dict.fromkeys(FINGERS,.012),action=Mock(),record=Mock())
 s.spin=Mock(side_effect=lambda _:s.joints.update(dict.fromkeys(FINGERS,.015)))
 CameraSkills.set_gripper(s,.015)
 assert s.spin.call_count==1 and s.action.call_count==1
