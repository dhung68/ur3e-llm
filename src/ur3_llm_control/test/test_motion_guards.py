"""A failed grasp must never issue a transport trajectory."""
from types import SimpleNamespace
from unittest.mock import Mock

from ur3_llm_control.skills import Skills


def context(held_object, verified):
    return SimpleNamespace(
        held_object=held_object, holding_verified=verified,
        config={"zones": {"zone_a": {}}},
        check_carried=Mock(side_effect=AssertionError("Unexpected physical transport check")),
        cartesian=Mock(side_effect=AssertionError("Unexpected motion")),
        record=Mock(), result=Mock(side_effect=lambda skill, success, reason: (success, reason)),
    )


def test_attached_but_unverified_object_cannot_be_placed():
    node = context("red_cube", False)
    success, reason = Skills.place(node, "red_cube", "zone_a")
    assert not success and "physically verified" in reason
    node.cartesian.assert_not_called()
    node.check_carried.assert_not_called()


def test_wrong_object_cannot_be_placed_even_with_a_verified_grasp():
    node = context("yellow_cube", True)
    success, reason = Skills.place(node, "red_cube", "zone_a")
    assert not success and "physically verified" in reason
    node.cartesian.assert_not_called()


def test_loss_observed_before_transport_stops_motion():
    node = context("red_cube", True)
    node.check_carried.side_effect = RuntimeError("Cube dropped in Gazebo")
    success, reason = Skills.place(node, "red_cube", "zone_a")
    assert not success and "dropped" in reason
    node.cartesian.assert_not_called()


def test_home_refuses_an_existing_unverified_attachment():
    node = context(None, False)
    node.contact_object = "red_cube"
    node.sync_scene = Mock(side_effect=AssertionError("Unexpected scene/motion operation"))
    node.move_joints = Mock(side_effect=AssertionError("Unexpected home motion"))
    success, reason = Skills.home(node)
    assert not success and "unresolved" in reason
    node.sync_scene.assert_not_called()
    node.move_joints.assert_not_called()


def test_occupied_destination_refuses_transport():
    node = context("red_cube", True)
    node.check_carried.side_effect = None
    node.verify_zone_empty = Mock(side_effect=RuntimeError("Target zone zone_a is occupied"))
    success, reason = Skills.place(node, "red_cube", "zone_a")
    assert not success and "occupied" in reason
    node.cartesian.assert_not_called()


def test_empty_zone_uses_positions_and_sizes_not_object_color():
    from ur3_llm_control.skills import Skills
    cube = lambda x, y: SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))
    node = SimpleNamespace(
        config={"zones": {"zone_b": {"placement_cube_center": [.38, 0., .095], "size_xy": [.08,.08]}},
                "cubes": {name: {"size": [.03,.03,.03]} for name in ["red_cube", "yellow_cube", "blue_cube"]}},
        provider=SimpleNamespace(get=lambda name: {"red_cube":cube(.38,0), "yellow_cube":cube(.22,0), "blue_cube":cube(.22,.14)}[name]),
        record=Mock(),
    )
    Skills.verify_zone_empty(node, "red_cube", "zone_b")  # held object excluded
    import pytest
    with pytest.raises(RuntimeError, match="red_cube"):
        Skills.verify_zone_empty(node, "blue_cube", "zone_b")
