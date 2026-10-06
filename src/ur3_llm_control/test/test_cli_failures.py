from types import SimpleNamespace
from unittest.mock import Mock
import json

from ur3_llm_control import cli
from ur3_llm_control.skills import SkillResult, Skills


def test_exception_after_successful_place_returns_nonzero(monkeypatch, tmp_path):
    outcome = SkillResult(True, "place", "physical placement succeeded", None, False, None, False)
    node = SimpleNamespace(
        config={"cubes": {"red_cube": {"size": [.03,.03,.03]}},
                "zones": {"zone_a": {"placement_cube_center": [.38,-.14,.095], "size_xy": [.08,.08]}}},
        spin=Mock(), record=Mock(), verify_zone_empty=Mock(),
        home=Mock(return_value=outcome), pick=Mock(return_value=outcome), place=Mock(return_value=outcome),
        provider=SimpleNamespace(get=Mock(side_effect=RuntimeError("Stale observation in final verification"))),
        destroy_node=Mock(),
    )
    monkeypatch.setattr(cli, "Skills", lambda *args, **kwargs: node)
    monkeypatch.setattr(cli.rclpy, "init", Mock())
    monkeypatch.setattr(cli.rclpy, "ok", lambda: False)
    path = tmp_path / "trial"
    monkeypatch.setattr("sys.argv", ["run_skills", "--evidence", str(path)])
    assert cli.main() == 1
    assert "Stale" in json.loads((path / "fatal_error.json").read_text())["error"]
    node.destroy_node.assert_called_once()


def test_camera_adjustment_timeout_still_captures_current_gui(monkeypatch, tmp_path):
    from ur3_llm_control import screenshots
    from geometry_msgs.msg import Pose
    monkeypatch.setattr(screenshots, "aim_gazebo", Mock(side_effect=RuntimeError("Camera service timed out")))
    capture = Mock(return_value=["actual_gazebo.png", "actual_rviz.png"])
    monkeypatch.setattr(screenshots, "capture_windows", capture)
    node = SimpleNamespace(
        screenshots=True, evidence=tmp_path, active_object="blue_cube", events=[],
        spin=Mock(), tcp_pose=lambda: Pose(),
        provider=SimpleNamespace(get=lambda name: SimpleNamespace(pose=Pose())),
        record=Mock(),
    )
    Skills.snapshot(node, "released_and_retreated")
    capture.assert_called_once()
    stages=[call.args[0] for call in node.record.call_args_list]
    assert "gui_camera_unavailable" in stages and "screenshots" in stages
    assert "screenshot_unavailable" not in stages
